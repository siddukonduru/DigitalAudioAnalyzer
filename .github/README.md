# DigitalAudioAnalyzer

A 6-channel USB audio analyzer for measuring the speaker outputs of a car infotainment head unit.

The hardware (RP2040 + 2 × PCM1865 ADCs) shows up on a laptop as a standard **USB Audio 2.0 microphone** (6 channels, 24-bit, 48 kHz). No driver is needed. The Windows app measures each channel live.

This repository contains:

| Folder | What |
|---|---|
| [`firmware/`](../firmware) | RP2040 firmware (C, pico-sdk 2.1.1 + TinyUSB) and a ready-to-flash `AudioAnalyzer_PC_fw.uf2` (v2.7) |
| [`pc_app/`](../pc_app) | Windows analyzer app (Python, PySide6 + pyqtgraph) |

## PC app

Per-channel measurements: level (Vrms / dBV / dBu / dBFS), gain relative to a reference channel, peak, fundamental frequency, THD, THD+N, SNR, DC offset and clipping detection. The app also shows waveform and spectrum views, logs CSV (one row per 100 ms), records 6-channel 24-bit WAV, and has per-channel calibration.

**Run from source** (Windows, Python 3.10+ with "Add python.exe to PATH" ticked):

```bat
cd pc_app
run.bat
```

`run.bat` installs the packages from `requirements.txt` the first time, then starts `app.py`. To run it by hand:

```bat
pip install -r requirements.txt
python app.py
```

**Build a standalone exe:** run `build_exe.bat`. It produces `dist\HU_Audio_Analyzer.exe` with PyInstaller.

**Windows audio stuck?** `fix_windows_audio.bat` restarts the Windows audio services. It asks for administrator permission.

## Firmware

### Flashing

1. Connect the board over USB-C. A blank board appears as an **RPI-RP2** drive. If it doesn't, hold **BOOT**, tap **RESET**, then release **BOOT**.
2. Copy `firmware/AudioAnalyzer_PC_fw.uf2` onto the drive.
3. The board reboots as **"HU Audio USB Analyzer 6ch"**.

If the app reports fewer than 6 channels, open Windows Sound settings → the analyzer input → Advanced, and set the default format to **6 channel, 24 bit, 48000 Hz**.

### Building

```sh
cd firmware
mkdir build && cd build
cmake -G Ninja -DPICO_SDK_PATH=<path-to-pico-sdk> ..
ninja
```

The build produces `audio_analyzer_fw.uf2`.

### Hardware notes

- I²C1 on GPIO18/19 (SDA/SCL) controls the two PCM1865s at addresses 0x4A and 0x4B.
- TDM input: BCK on GPIO20, LRCK on GPIO22, DOUT on GPIO24 (ADC1, ch 1–4) and GPIO25 (ADC2, ch 5–6). Captured by PIO + DMA.
- Status LED on GPIO8: solid = streaming, slow blink = idle, fast blink = ADC fault.
- Debug UART1 on GPIO4/5, 115200 8N1.
