"""
HU Audio USB Analyzer - Windows desktop app
6-channel level / waveform / spectrum / frequency / THD / THD+N / SNR meter for the
AudioAnalyzer_PC board (RP2040 + 2x PCM1865, USB Audio Class 2, 6 ch 24-bit 48 kHz).

Run:  python app.py        (needs: PySide6 pyqtgraph sounddevice numpy)
"""
import sys, time, csv, os, threading, queue, json
import numpy as np
import sounddevice as sd
try:
    import serial, serial.tools.list_ports
except ImportError:
    serial = None
import pyqtgraph as pg
from PySide6 import QtWidgets, QtCore, QtGui
from dsp import Analyzer, FS

N_CH = 6
DEVICE_HINT = "HU Audio USB Analyzer"
APP_DIR = os.path.dirname(sys.executable) if getattr(sys, "frozen", False) else os.path.dirname(os.path.abspath(__file__))
SETTINGS_FILE = os.path.join(APP_DIR, "analyzer_settings.json")
CONNECT_LOG = os.path.join(APP_DIR, "analyzer_connect_log.txt")
DEFAULT_FS_VRMS = 66.0      # Vrms at the HU terminals for a digital full-scale sine (rev A divider 47k/4.7k + ADC 10k, PGA 0 dB)
CH_NAMES = [f"CH{i+1}" for i in range(N_CH)]
DEFAULT_SILENT_MV = 1.2          # channel counts as "silent" below this level (mV rms at the HU terminals)


HOSTAPI_ORDER = ["Windows WASAPI", "MME", "Windows DirectSound", "Windows WDM-KS"]   # WDM-KS last: it fails with
# ERROR_BUSY (GLE 0xAA) whenever the Windows audio engine already has the device open


def find_devices():
    """All host-API copies of the analyzer, best first (6-ch WASAPI, then MME, DirectSound, WDM-KS)."""
    out = []
    for i, d in enumerate(sd.query_devices()):
        # MME truncates names to 31 chars ("Microphone (HU Audio USB Analyz"), so match on a short hint
        if "hu audio usb" in d['name'].lower() and d['max_input_channels'] >= 1:
            ha = sd.query_hostapis(d['hostapi'])['name']
            rank = HOSTAPI_ORDER.index(ha) if ha in HOSTAPI_ORDER else len(HOSTAPI_ORDER)
            out.append(((d['max_input_channels'] < N_CH), rank, i, d, ha))
    out.sort(key=lambda t: (t[0], t[1]))
    return [(i, d, ha) for _, _, i, d, ha in out]


class Capture(threading.Thread):
    """Pulls audio from the device into a ring buffer."""

    def __init__(self, dev_index, channels, seconds=4.0):
        super().__init__(daemon=True)
        self.dev = dev_index
        self.ch = channels
        self.n = int(FS * seconds)
        self.buf = np.zeros((self.n, channels), dtype=np.float32)
        self.pos = 0
        self.total = 0
        self.lock = threading.Lock()
        self.stream = None
        self.error = None
        self.record_q = None
        self.mode = ""
        self.open_errors = []
        self.dtype = 'float32'

    def _cb_raw24(self, indata, frames, t, status):
        b = np.frombuffer(indata, dtype=np.uint8).reshape(frames, self.ch, 3)
        v = b[:, :, 0].astype(np.int32) | (b[:, :, 1].astype(np.int32) << 8) | (b[:, :, 2].astype(np.int32) << 16)
        v = np.where(v >= (1 << 23), v - (1 << 24), v)
        self._cb((v.astype(np.float32) / 8388608.0), frames, t, status)

    def _cb(self, indata, frames, t, status):
        if status:
            self.error = str(status)
        if indata.dtype != np.float32:
            indata = indata.astype(np.float32) / 2147483648.0
        with self.lock:
            end = self.pos + frames
            if end <= self.n:
                self.buf[self.pos:end] = indata
            else:
                k = self.n - self.pos
                self.buf[self.pos:] = indata[:k]
                self.buf[:frames - k] = indata[k:]
            self.pos = end % self.n
            self.total += frames
        if self.record_q is not None:
            self.record_q.put(indata.copy())

    def open(self, hostapi=""):
        # Shared mode with the RAW stream option: Windows skips its enhancement/effects processing (APOs, Dolby,
        # Waves, AGC...) for this stream. Firmware v2.7+ reports a hardware volume control, so the Windows input
        # volume slider does not scale the data either. (Exclusive mode was dropped: Windows refuses it for this
        # board's channel layout and it would lock other apps out of the device.)
        attempts = []
        if 'WASAPI' in hostapi:
            try:
                ws = sd.WasapiSettings()
                ws._streaminfo.streamOption = 1      # eStreamOptionRaw (Windows 8.1+)
                attempts.append(("shared raw", dict(extra_settings=ws)))
            except Exception:
                pass
        attempts.append(("shared", {}))
        if 'WASAPI' in hostapi:
            try:
                attempts.append(("shared+convert", dict(extra_settings=sd.WasapiSettings(auto_convert=True))))
            except TypeError:
                pass
        errs = []
        for name, extra in attempts:
            for dtype in ('float32', 'int32'):
                try:
                    self.dtype = dtype
                    if dtype == 'int24':   # the board's native format (24-bit packed): needed for WASAPI exclusive
                        self.stream = sd.RawInputStream(device=self.dev, channels=self.ch, samplerate=FS, dtype='int24',
                                                        blocksize=0, latency='high', callback=self._cb_raw24, **extra)
                    else:
                        self.stream = sd.InputStream(device=self.dev, channels=self.ch, samplerate=FS, dtype=dtype,
                                                     blocksize=0, latency='high', callback=self._cb, **extra)
                    self.stream.start()
                    self.mode = name
                    self.open_errors = errs
                    return
                except Exception as e:
                    errs.append(f"{name}/{dtype}: {e}")
                    try:
                        if self.stream:
                            self.stream.close()
                    except Exception:
                        pass
                    self.stream = None
        raise RuntimeError(" | ".join(errs))

    def latest(self, n):
        with self.lock:
            idx = (np.arange(self.pos - n, self.pos)) % self.n
            return self.buf[idx].copy()

    def close(self):
        if self.stream:
            self.stream.stop(); self.stream.close(); self.stream = None


class WavWriter(threading.Thread):
    def __init__(self, path, channels, q):
        super().__init__(daemon=True)
        import wave
        self.w = wave.open(path, 'wb')
        self.w.setnchannels(channels); self.w.setsampwidth(3); self.w.setframerate(FS)
        self.q = q; self.stop = False

    def run(self):
        while not self.stop or not self.q.empty():
            try:
                blk = self.q.get(timeout=0.2)
            except queue.Empty:
                continue
            i = np.clip(blk * 8388607.0, -8388608, 8388607).astype(np.int32)
            b = i.astype('<i4').tobytes()
            arr = np.frombuffer(b, dtype=np.uint8).reshape(-1, 4)[:, :3]
            self.w.writeframes(arr.tobytes())
        self.w.close()


class _Signals(QtCore.QObject):
    connected = QtCore.Signal(object, str)


class MainWindow(QtWidgets.QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("HU Audio USB Analyzer")
        self.resize(1400, 860)
        self.settings = self.load_settings()
        self.cap = None
        self.an = Analyzer(16384)
        self.results = [dict() for _ in range(N_CH)]
        self.log_file = None
        self.rec = None
        self.sig = _Signals(); self.sig.connected.connect(self._on_connected)
        self.build_ui()
        self.lbl_dev.setWordWrap(True)
        self.timer = QtCore.QTimer(self); self.timer.timeout.connect(self.update); self.timer.start(100)
        QtCore.QTimer.singleShot(200, self.connect_device)

    # ---------------- settings
    def load_settings(self):
        s = {"fs_vrms": DEFAULT_FS_VRMS, "cal": [1.0] * N_CH, "unit": "Vrms", "names": CH_NAMES[:], "silent_mv": DEFAULT_SILENT_MV}
        try:
            with open(SETTINGS_FILE) as f:
                s.update(json.load(f))
        except Exception:
            pass
        return s

    def save_settings(self):
        try:
            with open(SETTINGS_FILE, 'w') as f:
                json.dump(self.settings, f, indent=1)
        except Exception:
            pass

    # ---------------- UI
    def build_ui(self):
        pg.setConfigOptions(antialias=False, background='k', foreground='w')
        central = QtWidgets.QWidget(); self.setCentralWidget(central)
        root = QtWidgets.QVBoxLayout(central)

        top = QtWidgets.QHBoxLayout(); root.addLayout(top)
        self.lbl_dev = QtWidgets.QLabel("Device: (searching...)"); top.addWidget(self.lbl_dev, 1)
        self.btn_conn = QtWidgets.QPushButton("Reconnect"); self.btn_conn.clicked.connect(self.connect_device); top.addWidget(self.btn_conn)
        top.addWidget(QtWidgets.QLabel("Full scale ="))
        self.spin_fs = QtWidgets.QDoubleSpinBox(); self.spin_fs.setRange(0.1, 500); self.spin_fs.setDecimals(2)
        self.spin_fs.setValue(self.settings["fs_vrms"]); self.spin_fs.setSuffix(" Vrms"); self.spin_fs.valueChanged.connect(self.on_fs)
        top.addWidget(self.spin_fs)
        top.addWidget(QtWidgets.QLabel("Silent <"))
        self.spin_sil = QtWidgets.QDoubleSpinBox(); self.spin_sil.setRange(0.1, 1000); self.spin_sil.setDecimals(2)
        self.spin_sil.setSingleStep(0.1); self.spin_sil.setSuffix(" mV"); self.spin_sil.setValue(float(self.settings.get("silent_mv", DEFAULT_SILENT_MV)))
        self.spin_sil.setToolTip("Channels below this level show 'silent' and no Freq/THD/SNR. Noise floor is ~0.4-0.6 mV; "
                                 "CH1->CH2 crosstalk can reach ~1 mV with a 9 V signal on CH1.")
        self.spin_sil.valueChanged.connect(lambda v: (self.settings.__setitem__("silent_mv", v), self.save_settings()))
        top.addWidget(self.spin_sil)
        self.cmb_unit = QtWidgets.QComboBox(); self.cmb_unit.addItems(["Vrms", "dBV", "dBu", "dBFS"]); self.cmb_unit.setCurrentText(self.settings["unit"])
        self.cmb_unit.currentTextChanged.connect(self.on_unit); top.addWidget(self.cmb_unit)
        self.btn_log = QtWidgets.QPushButton("Start CSV log"); self.btn_log.setCheckable(True); self.btn_log.toggled.connect(self.toggle_log); top.addWidget(self.btn_log)
        self.btn_rec = QtWidgets.QPushButton("Record WAV"); self.btn_rec.setCheckable(True); self.btn_rec.toggled.connect(self.toggle_rec); top.addWidget(self.btn_rec)
        self.btn_board = QtWidgets.QPushButton("Board status"); self.btn_board.clicked.connect(self.board_status); top.addWidget(self.btn_board)
        top.addWidget(QtWidgets.QLabel("Gain ref:"))
        self.cmb_ref = QtWidgets.QComboBox(); self.cmb_ref.addItem("none"); self.cmb_ref.addItems(self.settings["names"])
        self.cmb_ref.setCurrentIndex(int(self.settings.get("gain_ref", 0)))
        self.cmb_ref.currentIndexChanged.connect(self.on_ref); self.cmb_ref.setToolTip("Reference channel for the Gain column: every channel shows 20*log10(level / reference level)")
        top.addWidget(self.cmb_ref)

        # meter table
        self.table = QtWidgets.QTableWidget(N_CH, 11)
        self.table.setHorizontalHeaderLabels(["Channel", "Level", "Gain dB", "Peak", "Freq (Hz)", "THD %", "THD+N %", "SNR dB", "DC", "Cal x", "Status"])
        self.table.verticalHeader().setVisible(False)
        self.table.horizontalHeader().setSectionResizeMode(QtWidgets.QHeaderView.Stretch)
        self.table.setMaximumHeight(240)
        f = QtGui.QFont("Consolas", 11)
        for r in range(N_CH):
            for c in range(11):
                it = QtWidgets.QTableWidgetItem("")
                it.setFont(f)
                if c not in (0, 9):
                    it.setFlags(it.flags() & ~QtCore.Qt.ItemIsEditable)
                self.table.setItem(r, c, it)
            self.table.item(r, 0).setText(self.settings["names"][r])
            self.table.item(r, 9).setText(f'{self.settings["cal"][r]:.4f}')
        self.table.itemChanged.connect(self.on_table_edit)
        root.addWidget(self.table)

        # plots
        plots = QtWidgets.QHBoxLayout(); root.addLayout(plots, 1)
        self.scope = pg.PlotWidget(title="Waveform (last 20 ms)"); self.scope.setLabel('left', 'V'); self.scope.setLabel('bottom', 'ms')
        self.scope.addLegend(offset=(10, 10)); plots.addWidget(self.scope, 1)
        self.spec = pg.PlotWidget(title="Spectrum (dBFS)"); self.spec.setLogMode(x=True, y=False); self.spec.setYRange(-140, 5)
        self.spec.setLabel('bottom', 'Hz'); self.spec.setLabel('left', 'dBFS'); self.spec.addLegend(offset=(10, 10)); plots.addWidget(self.spec, 1)
        colors = ['#ff5555', '#55ff55', '#5599ff', '#ffff55', '#ff55ff', '#55ffff']
        self.scope_curves = [self.scope.plot(pen=pg.mkPen(colors[i], width=1), name=CH_NAMES[i]) for i in range(N_CH)]
        self.spec_curves = [self.spec.plot(pen=pg.mkPen(colors[i], width=1), name=CH_NAMES[i]) for i in range(N_CH)]
        self.scope.setMouseEnabled(x=False, y=True)

        ctl = QtWidgets.QHBoxLayout(); root.addLayout(ctl)
        ctl.addWidget(QtWidgets.QLabel("Show:"))
        self.chk = []
        for i in range(N_CH):
            c = QtWidgets.QCheckBox(CH_NAMES[i]); c.setChecked(True); c.setStyleSheet(f"color:{colors[i]}"); ctl.addWidget(c); self.chk.append(c)
        ctl.addStretch(1)
        self.lbl_status = QtWidgets.QLabel(""); ctl.addWidget(self.lbl_status)

    # ---------------- device
    def connect_device(self):
        """Opens the board on a worker thread so a slow/failed driver call can't freeze the window."""
        if getattr(self, "_connecting", False):
            return
        self._connecting = True
        self.btn_conn.setEnabled(False)
        self.lbl_dev.setText("Device: connecting...")
        if self.cap:
            self.cap.close(); self.cap = None
        # PortAudio must be (re)initialised on the GUI thread: MME / DirectSound / WASAPI need COM on the calling
        # thread, and initialising from a worker leaves only WDM-KS (which Windows keeps busy)
        try:
            sd._terminate(); sd._initialize()
        except Exception:
            pass
        threading.Thread(target=self._connect_worker, daemon=True).start()

    def _connect_worker(self):
        log = []
        cap, text = None, ""
        com = None
        try:
            if sys.platform == "win32":
                import ctypes
                com = ctypes.windll.ole32
                com.CoInitializeEx(None, 0)          # COINIT_MULTITHREADED, needed to open WASAPI from this thread
        except Exception:
            com = None
        # every step is appended to the log file immediately, so a hang shows exactly where it stopped
        def step(msg):
            log.append(msg)
            try:
                with open(CONNECT_LOG, "a") as f:
                    f.write(time.strftime("%H:%M:%S ") + msg + "\n")
            except Exception:
                pass
        try:
            open(CONNECT_LOG, "w").write(time.strftime("%Y-%m-%d %H:%M:%S") + " connect started\n")
        except Exception:
            pass
        log.clear()
        step("COM ports: " + (", ".join(f"{p.device} vid={p.vid} pid={p.pid} '{p.description}'"
                                      for p in serial.tools.list_ports.comports()) if serial else "pyserial missing"))
        res = {}
        def bq():
            try:
                res['r'] = self.board_query(["v", "s", "l"], wait=1.5)
            except Exception as e:
                res['r'] = (None, f"error: {e}")
        # the board may be dropping off USB and coming back: keep retrying the debug port for ~25 s
        t0 = time.time()
        while True:
            res.clear()
            t = threading.Thread(target=bq, daemon=True); t.start(); t.join(12)
            ok = (not t.is_alive()) and res.get('r') and res['r'][0] and not str(res['r'][1]).startswith("could not open") \
                 and "not found" not in str(res['r'][1])
            if ok or time.time() - t0 > 25 or t.is_alive():
                break
            step(f"debug port attempt failed: {res.get('r')}")
            time.sleep(0.3)
        if t.is_alive():
            step("board debug port: NO ANSWER after 12 s (port open/write is hanging -> board USB stuck?)")
        else:
            port, btxt = res['r']
            step(f"---- board debug port [{port}] ----\n{btxt}\n---- end board ----")
        try:
            step("host APIs: " + ", ".join(f"{h['name']}({len(h['devices'])} dev)" for h in sd.query_hostapis()))
            cands = find_devices()
            for i, d in enumerate(sd.query_devices()):
                log.append(f"[{i}] {sd.query_hostapis(d['hostapi'])['name']:20s} in={d['max_input_channels']} {d['name']}")
            if not cands:
                text = "Device: NOT FOUND - plug in the analyzer board (it shows up as a 6-channel microphone)"
            fails = []
            for idx, d, ha in cands:
                ch = min(N_CH, d['max_input_channels'])
                c = Capture(idx, ch)
                step(f"trying [{idx}] {ha} ({d['max_input_channels']} ch) ...")
                try:
                    c.open(ha)
                    cap = c
                    text = f"Device: {d['name']}  [{ha}, {c.mode}, {ch} ch, {FS} Hz]"
                    if c.mode != "shared raw":
                        step(f"raw mode not available ({c.mode}); open errors: {'; '.join(c.open_errors)}")
                        text += ("  -  NOTE: Windows raw mode not available - keep Audio enhancements OFF for this device "
                                 "(Sound settings > the device > Audio enhancements, and Dolby/OEM audio tools).")
                    if ch < N_CH:
                        text += ("  - only %d ch exposed: set the device to 6 channels 24-bit 48000 Hz in Windows Sound "
                                 "settings > Recording > Properties > Advanced" % ch)
                    step(f"OPENED [{idx}] {ha} {c.mode} {c.dtype} {ch}ch")
                    break
                except Exception as e:
                    fails.append(f"{ha}: {e}")
                    step(f"FAILED [{idx}] {ha}: {e}")
            n_engine = sum(1 for d in sd.query_devices()
                           if sd.query_hostapis(d['hostapi'])['name'] in ("MME", "Windows DirectSound", "Windows WASAPI"))
            if cap is None and n_engine == 0:
                text = ("Device: Windows' audio service is not listing ANY devices (only raw WDM-KS is visible, and the "
                        "board is held busy there). Close all copies of this app in Task Manager, then restart the "
                        "'Windows Audio' service (services.msc) or reboot, and press Reconnect.")
            elif cands and cap is None:
                text = "Device: found but could not open on any Windows audio API - see analyzer_connect_log.txt. " + \
                       " || ".join(fails)[:400]
        except Exception as e:
            text = f"Device: error while connecting: {e}"
            log.append(text)
        try:
            with open(CONNECT_LOG, "a") as f:
                f.write("==== full log ====\n" + "\n".join(log) + "\nRESULT: " + text + "\n")
        except Exception:
            pass
        self.sig.connected.emit(cap, text)

    def _on_connected(self, cap, text):
        self.cap = cap
        self.lbl_dev.setText(text)
        self.lbl_dev.setToolTip(text)
        self._connecting = False
        self.btn_conn.setEnabled(True)

    # ---------------- board debug port (USB virtual COM, same cable)
    USB_VID, USB_PID = 0xCAFE, 0x4A06

    def find_com_port(self):
        if serial is None:
            return None
        for p in serial.tools.list_ports.comports():
            if p.vid == self.USB_VID and p.pid == self.USB_PID:
                return p.device
        for p in serial.tools.list_ports.comports():
            if p.description and "HU Analyzer" in p.description:
                return p.device
        return None

    def board_query(self, cmds, wait=1.0):
        port = self.find_com_port()
        if not port:
            return None, "Board debug port not found (needs firmware v1.1+ and the pyserial package)."
        try:
            with serial.Serial(port, 115200, timeout=0.2, write_timeout=1.0) as ser:
                time.sleep(0.2)
                ser.reset_input_buffer()
                out = ""
                for c in cmds:
                    ser.write((c + "\n").encode())
                    t0 = time.time()
                    while time.time() - t0 < wait:
                        chunk = ser.read(4096)
                        if chunk:
                            out += chunk.decode(errors="replace")
                        elif out and time.time() - t0 > 0.3:
                            break
                return port, out
        except Exception as e:
            return port, f"could not open {port}: {e}"

    def board_status(self):
        port, text = self.board_query(["v", "s", "l"])
        dlg = QtWidgets.QDialog(self); dlg.setWindowTitle(f"Board status  [{port or 'no port'}]"); dlg.resize(900, 600)
        lay = QtWidgets.QVBoxLayout(dlg)
        box = QtWidgets.QPlainTextEdit(); box.setReadOnly(True); box.setFont(QtGui.QFont("Consolas", 10)); box.setPlainText(text); lay.addWidget(box)
        row = QtWidgets.QHBoxLayout(); lay.addLayout(row)
        inp = QtWidgets.QLineEdit(); inp.setPlaceholderText("command: s status, l boot log, r re-init ADCs, g <dB> PGA gain, h help"); row.addWidget(inp, 1)
        def send():
            _, t = self.board_query([inp.text().strip()])
            box.appendPlainText(f"\n> {inp.text().strip()}\n{t}")
            inp.clear()
        btn = QtWidgets.QPushButton("Send"); btn.clicked.connect(send); inp.returnPressed.connect(send); row.addWidget(btn)
        dlg.exec()

    # ---------------- callbacks
    def on_fs(self, v):
        self.settings["fs_vrms"] = v; self.save_settings()

    def on_unit(self, u):
        self.settings["unit"] = u; self.save_settings()

    def on_ref(self, i):
        self.settings["gain_ref"] = int(i); self.save_settings()

    def on_table_edit(self, item):
        r, c = item.row(), item.column()
        if c == 9:
            try:
                self.settings["cal"][r] = float(item.text())
            except ValueError:
                item.setText(f'{self.settings["cal"][r]:.4f}')
            self.save_settings()
        elif c == 0:
            self.settings["names"][r] = item.text()
            self.cmb_ref.setItemText(r + 1, item.text())
            self.save_settings()

    def fmt_level(self, vrms):
        u = self.settings["unit"]
        fs = self.settings["fs_vrms"]
        if u == "Vrms":
            return f"{vrms:8.4f} V"
        if u == "dBV":
            return f"{20*np.log10(max(vrms,1e-9)):7.2f} dBV"
        if u == "dBu":
            return f"{20*np.log10(max(vrms,1e-9)/0.7746):7.2f} dBu"
        return f"{20*np.log10(max(vrms/fs,1e-9)):7.2f} dBFS"

    def toggle_log(self, on):
        if on:
            path = QtWidgets.QFileDialog.getSaveFileName(self, "CSV log file", "hu_audio_log.csv", "CSV (*.csv)")[0]
            if not path:
                self.btn_log.setChecked(False); return
            self.log_file = open(path, 'w', newline='')
            self.log_w = csv.writer(self.log_file)
            hdr = ["time"]
            for i in range(N_CH):
                hdr += [f"ch{i+1}_vrms", f"ch{i+1}_gain_db", f"ch{i+1}_peak_v", f"ch{i+1}_freq_hz", f"ch{i+1}_thd_pct", f"ch{i+1}_thdn_pct", f"ch{i+1}_snr_db"]
            self.log_w.writerow(hdr)
            self.btn_log.setText("Stop CSV log")
        else:
            if self.log_file:
                self.log_file.close(); self.log_file = None
            self.btn_log.setText("Start CSV log")

    def toggle_rec(self, on):
        if on:
            if not self.cap:
                self.btn_rec.setChecked(False); return
            path = QtWidgets.QFileDialog.getSaveFileName(self, "WAV file", time.strftime("hu_audio_%Y%m%d_%H%M%S.wav"), "WAV (*.wav)")[0]
            if not path:
                self.btn_rec.setChecked(False); return
            q = queue.Queue()
            self.rec = WavWriter(path, self.cap.ch, q); self.rec.start()
            self.cap.record_q = q
            self.btn_rec.setText("Stop recording")
        else:
            if self.cap:
                self.cap.record_q = None
            if self.rec:
                self.rec.stop = True; self.rec = None
            self.btn_rec.setText("Record WAV")

    # ---------------- periodic update
    def update(self):
        if not self.cap:
            return
        if self.cap.total < self.an.n:
            return
        x = self.cap.latest(self.an.n)          # (n, ch)
        fs_v = self.settings["fs_vrms"]
        k = fs_v * np.sqrt(2)                   # volts per full-scale unit (peak)
        tms = np.arange(int(FS * 0.02)) / FS * 1e3
        row = [time.strftime("%Y-%m-%d %H:%M:%S")]
        xs = []
        for i in range(self.cap.ch):
            xi = x[:, i] * self.settings["cal"][i]
            xs.append(xi)
            self.results[i] = self.an.analyze(xi)
        ref = int(self.settings.get("gain_ref", 0)) - 1          # -1 = none
        ref_rms = self.results[ref]['rms'] if 0 <= ref < self.cap.ch else 0.0
        for i in range(self.cap.ch):
            xi = xs[i]
            r = self.results[i]
            vrms = r['rms'] * k
            vpk = r['peak'] * k
            if ref_rms > 1e-7 and r['rms'] > 1e-7:
                gain = 20 * np.log10(r['rms'] / ref_rms)
                gain_s = "  ref  " if i == ref else f"{gain:+7.2f}"
            else:
                gain, gain_s = float('nan'), "-"
            self.table.item(i, 1).setText(self.fmt_level(vrms))
            self.table.item(i, 2).setText(gain_s)
            self.table.item(i, 3).setText(f"{vpk:8.3f} V")
            # silent = below the 'Silent <' level (default 1.2 mV at the HU): only noise / crosstalk from other channels
            silent = vrms < float(self.settings.get("silent_mv", DEFAULT_SILENT_MV)) / 1000.0
            valid = (r['freq'] > 0) and not silent
            self.table.item(i, 4).setText(f"{r['freq']:9.2f}" if valid else "-")
            self.table.item(i, 5).setText(f"{r['thd']*100:7.4f}" if valid else "-")
            self.table.item(i, 6).setText(f"{r['thdn']*100:7.4f}" if valid else "-")
            self.table.item(i, 7).setText(f"{r['snr']:6.1f}" if valid else "-")
            self.table.item(i, 8).setText(f"{r['dc']*k:+.3f} V")
            st = "CLIP" if r['clip'] else ("silent" if silent else "ok")
            self.table.item(i, 10).setText(st)
            self.table.item(i, 10).setForeground(QtGui.QColor('#ff4444' if r['clip'] else '#44ff44'))
            row += [f"{vrms:.5f}", "" if gain != gain else f"{gain:.3f}", f"{vpk:.4f}"] + \
                   ([f"{r['freq']:.3f}", f"{r['thd']*100:.5f}", f"{r['thdn']*100:.5f}", f"{r['snr']:.2f}"] if valid else ["", "", "", ""])
            if self.chk[i].isChecked():
                seg = xi[-len(tms):] * k
                self.scope_curves[i].setData(tms, seg)
                f, m = self.an.spectrum(xi)
                self.spec_curves[i].setData(f[1:], m[1:])
            else:
                self.scope_curves[i].setData([], []); self.spec_curves[i].setData([], [])
        if self.log_file:
            self.log_w.writerow(row); self.log_file.flush()
        err = f"  stream status: {self.cap.error}" if self.cap.error else ""
        self.lbl_status.setText(f"samples: {self.cap.total}{err}")

    def closeEvent(self, e):
        if self.cap:
            self.cap.close()
        if self.log_file:
            self.log_file.close()
        if self.rec:
            self.rec.stop = True
        e.accept()


def main():
    app = QtWidgets.QApplication(sys.argv)
    app.setStyle("Fusion")
    w = MainWindow(); w.show()
    sys.exit(app.exec())


if __name__ == "__main__":
    main()
