"""Audio measurement DSP for the HU Audio USB Analyzer (6 ch, 48 kHz)."""
import numpy as np

FS = 48000


def flattop(n):
    # SRS flat-top window (amplitude accuracy < 0.01 dB)
    a = [0.21557895, 0.41663158, 0.277263158, 0.083578947, 0.006947368]
    k = np.arange(n)
    w = np.zeros(n)
    for i, ai in enumerate(a):
        w += ((-1) ** i) * ai * np.cos(2 * np.pi * i * k / (n - 1))
    return w


def hann(n):
    return 0.5 - 0.5 * np.cos(2 * np.pi * np.arange(n) / n)


class Analyzer:
    """Per-block analysis of one channel. x: float samples, full scale = +-1.0"""

    def __init__(self, n=16384, fs=FS):
        self.n = n
        self.fs = fs
        self.win = hann(n)
        self.win_gain = self.win.sum() / n           # coherent gain
        self.enbw = (self.win ** 2).sum() / (self.win.sum() ** 2) * n   # bins
        self.freqs = np.fft.rfftfreq(n, 1 / fs)

    def spectrum(self, x):
        """Return (freqs, magnitude in dBFS referenced to a full-scale sine)"""
        X = np.fft.rfft(x * self.win)
        mag = np.abs(X) / (self.n * self.win_gain) * 2  # peak amplitude of a sine
        mag[0] /= 2
        return self.freqs, 20 * np.log10(np.maximum(mag / 1.0, 1e-12))

    def analyze(self, x, n_harm=10):
        n = self.n
        x = x[-n:]
        res = {}
        res['dc'] = float(np.mean(x))
        xac = x - res['dc']
        res['rms'] = float(np.sqrt(np.mean(xac ** 2)))
        res['peak'] = float(np.max(np.abs(x)))
        res['clip'] = bool(res['peak'] > 0.999)

        X = np.fft.rfft(xac * self.win)
        P = np.abs(X) ** 2                       # power spectrum (arbitrary scale)
        P[0] = 0
        if len(P) > 3:
            P[:3] = 0                            # ignore < ~9 Hz
        k = int(np.argmax(P))
        if k < 2 or P[k] <= 0:
            res.update(freq=0.0, thd=0.0, thdn=0.0, snr=0.0, fund_rms=0.0, harmonics=[])
            return res
        # parabolic interpolation on log magnitude for frequency
        lm = np.log(np.maximum(P[k - 1:k + 2], 1e-30)) / 2
        d = 0.5 * (lm[0] - lm[2]) / (lm[0] - 2 * lm[1] + lm[2]) if (lm[0] - 2 * lm[1] + lm[2]) != 0 else 0
        f0 = (k + d) * self.fs / n
        res['freq'] = float(f0)

        # power in a +-w bin band around a frequency
        w = 4
        bins_per = self.fs / n

        def band_power(f):
            c = int(round(f / bins_per))
            lo, hi = max(1, c - w), min(len(P) - 1, c + w)
            return P[lo:hi + 1].sum(), (lo, hi)

        total = P.sum()
        pf, (flo, fhi) = band_power(f0)
        harm = []
        ph = 0.0
        Pn = P.copy()
        Pn[flo:fhi + 1] = 0
        for h in range(2, n_harm + 1):
            fh = f0 * h
            if fh >= self.fs / 2 - bins_per * (w + 1):
                break
            p, (lo, hi) = band_power(fh)
            harm.append((h, float(np.sqrt(p / pf)) if pf > 0 else 0.0))
            ph += p
            Pn[lo:hi + 1] = 0
        noise = Pn.sum()
        res['fund_rms'] = float(res['rms'] * np.sqrt(pf / total)) if total > 0 else 0.0
        res['thd'] = float(np.sqrt(ph / pf)) if pf > 0 else 0.0
        res['thdn'] = float(np.sqrt((ph + noise) / pf)) if pf > 0 else 0.0
        res['snr'] = float(10 * np.log10(pf / noise)) if noise > 0 else 0.0
        res['harmonics'] = harm
        return res


def db(x, ref=1.0):
    return 20 * np.log10(max(abs(x) / ref, 1e-12))
