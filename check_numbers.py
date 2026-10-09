import sys
import numpy as np
import pandas as pd

TRUE_BIAS = np.array([0.8, -0.5, 0.4])
wrap = lambda x: (x + 180) % 360 - 180

for f in sys.argv[1:]:
    df = pd.read_csv(f)
    t = df["timestamp_s"]
    late = t > 20
    rms = lambda s: np.sqrt((s[late] ** 2).mean())

    off = ~((df[["bias_x", "bias_y", "bias_z"]] - TRUE_BIAS).abs() < 0.08).all(axis=1)
    conv = t[off].max() if off.any() else 0.0
    settled = "never" if conv > t.max() - 1 else f"{conv:.1f} s"

    print(f"\n{f}")
    print(f"  bias within 0.08 deg/s from: {settled}")
    for a in ["roll", "pitch", "yaw"]:
        cf = wrap(df[f"cf_{a}"] - df[f"bno_{a}"])
        print(f"  {a:5s}  ESKF {rms(df['err_' + a]):.3f}   "
              f"before correction {rms(df['pred_err_' + a]):.3f}   "
              f"comp filter {rms(cf):.3f}  (deg RMS, t > 20 s)")
    for tt in [15, 29.5, 31, 50]:
        if tt <= t.max():
            r = df.iloc[(t - tt).abs().argmin()]
            print(f"  t={r.timestamp_s:5.1f}s  mag={int(r.cal_mag)}  "
                  f"yaw err {wrap(r.fused_yaw - r.bno_yaw):+.3f}  "
                  f"yaw sigma {r.yaw_sigma_deg:.2f} deg")
    if "noisy" in f:
        truth = {"roll": 15 * np.sin(0.3 * t),
                 "pitch": 10 * np.sin(0.2 * t + 0.5),
                 "yaw": 30 * np.sin(0.1 * t)}
        for a in ["roll", "pitch", "yaw"]:
            print(f"  {a:5s}  ESKF vs truth {rms(wrap(df['fused_' + a] - truth[a])):.3f}   "
                  f"BNO085 vs truth {rms(wrap(df['bno_' + a] - truth[a])):.3f}")
