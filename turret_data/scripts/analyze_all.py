"""
Three-way comparison: original → kD+alpha → SyncCANcoder
"""
import csv
import numpy as np

FILES = {
    "v1 kD=0 a=0.15 Fused": "turret_diag_20260328_170907.csv",
    "v2 kD=1 a=0.40 Fused": "turret_diag_20260328_173048.csv",
    "v3 kD=1 a=0.60 Sync ": "turret_diag_20260328_174931.csv",
}

def load(path):
    with open(path) as f:
        rows = list(csv.DictReader(f))
    return {k: np.array([float(r[k]) for r in rows]) for k in rows[0]}

def find_windows(cols, min_samples=50):
    n = len(cols["timestamp"])
    speed = np.sqrt(cols["vel_vx"]**2 + cols["vel_vy"]**2)
    stat = (speed < 0.05) & (np.abs(cols["vel_omega"]) < 0.05)
    windows, start = [], None
    for i in range(n):
        if stat[i] and start is None: start = i
        elif not stat[i] and start is not None:
            windows.append((start, i)); start = None
    if start is not None: windows.append((start, n))
    return [(s, e) for s, e in windows if (e - s) >= min_samples]

def window_metrics(cols, windows):
    results = []
    for s, e in windows:
        settle = min(50, (e - s) // 4)
        sl = slice(s + settle, e)
        t = cols["timestamp"][sl]
        if len(t) < 20: continue
        err = cols["turret_error"][sl]
        ae = np.abs(err)
        act = cols["turret_actual"][sl]
        sp = cols["turret_setpoint"][sl]
        results.append({
            "sp_p2p": np.ptp(sp), "sp_std": np.std(sp),
            "act_p2p": np.ptp(act), "act_std": np.std(act),
            "err_abs_mean": np.mean(ae), "err_abs_p95": np.percentile(ae, 95),
            "err_bias": np.mean(err), "err_std": np.std(err),
            "err_sign_chg": np.sum(np.diff(np.sign(err)) != 0) / max(len(err)-1, 1) * 100,
            "volt_std": np.std(cols["motor_voltage"][sl]),
            "heading_p2p": np.ptp(cols["pose_heading"][sl]),
            "raw_p2p": np.ptp(cols["raw_turret_angle"][sl]),
            "filt_p2p": np.ptp(cols["filtered_turret_angle"][sl]),
        })
    return results

def motion_metrics(cols):
    n = len(cols["timestamp"])
    speed = np.sqrt(cols["vel_vx"]**2 + cols["vel_vy"]**2)
    omega = np.abs(cols["vel_omega"])
    t0 = cols["timestamp"][0]
    settled = cols["timestamp"] > (t0 + 2.0)
    stat = settled & (speed < 0.05) & (omega < 0.05)
    move = settled & ((speed >= 0.1) | (omega >= 0.1))
    ae = np.abs(cols["turret_error"])
    filt_lag = cols["raw_turret_angle"] - cols["filtered_turret_angle"]
    sp = cols["turret_setpoint"]
    err = cols["turret_error"]
    m_idx = np.where(move)[0]
    lag, total = 0, 0
    for i in m_idx:
        if i == 0: continue
        d = sp[i] - sp[i-1]
        if abs(d) < 0.001: continue
        total += 1
        if np.sign(d) == np.sign(err[i]): lag += 1
    return {
        "stat_err": np.mean(ae[stat]) if np.sum(stat) > 0 else 0,
        "stat_p95": np.percentile(ae[stat], 95) if np.sum(stat) > 0 else 0,
        "move_err": np.mean(ae[move]) if np.sum(move) > 0 else 0,
        "move_p95": np.percentile(ae[move], 95) if np.sum(move) > 0 else 0,
        "filt_lag": np.mean(np.abs(filt_lag[move])) if np.sum(move) > 0 else 0,
        "lag_pct": 100 * lag / max(total, 1),
    }

# Load all
data = {}
for label, path in FILES.items():
    cols = load(path)
    wins = find_windows(cols)
    wm = window_metrics(cols, wins)
    mm = motion_metrics(cols)
    data[label] = {"wm": wm, "mm": mm, "cols": cols}

labels = list(FILES.keys())

def avg(label, key):
    return np.mean([m[key] for m in data[label]["wm"]])

# ── Three-way comparison ───────────────────────────────────────────────
print("=" * 90)
print("=== THREE-WAY COMPARISON: STATIONARY PERFORMANCE ===")
print("=" * 90)
print()
print(f"  {'Metric':30s}", end="")
for l in labels:
    print(f" {l:>17s}", end="")
print()
print(f"  {'-'*30}", end="")
for _ in labels:
    print(f" {'-'*17}", end="")
print()

def r3(name, key, unit="deg"):
    vals = [avg(l, key) for l in labels]
    print(f"  {name:30s}", end="")
    for v in vals:
        print(f" {v:15.4f} {unit[0]}", end="")
    # improvement from v1 to v3
    if vals[0] != 0:
        pct = (vals[-1] - vals[0]) / vals[0] * 100
        print(f"  ({pct:+.0f}%)", end="")
    print()

r3("heading p2p", "heading_p2p")
r3("raw_angle p2p", "raw_p2p")
r3("filtered_angle p2p", "filt_p2p")
r3("setpoint p2p", "sp_p2p")
r3("actual p2p", "act_p2p")
r3("actual std", "act_std")
r3("|error| mean", "err_abs_mean")
r3("|error| p95", "err_abs_p95")
r3("error bias (mean)", "err_bias")
r3("error std", "err_std")
r3("error sign changes %", "err_sign_chg")
r3("voltage std", "volt_std", "V")
print()

# ── Motion comparison ──────────────────────────────────────────────────
print("=" * 90)
print("=== THREE-WAY COMPARISON: MOTION TRACKING ===")
print("=" * 90)
print()
print(f"  {'Metric':30s}", end="")
for l in labels:
    print(f" {l:>17s}", end="")
print()
print(f"  {'-'*30}", end="")
for _ in labels:
    print(f" {'-'*17}", end="")
print()

def m3(name, key, unit="deg"):
    vals = [data[l]["mm"][key] for l in labels]
    print(f"  {name:30s}", end="")
    for v in vals:
        print(f" {v:15.4f} {unit[0]}", end="")
    if vals[0] != 0:
        pct = (vals[-1] - vals[0]) / vals[0] * 100
        print(f"  ({pct:+.0f}%)", end="")
    print()

m3("stationary |error| mean", "stat_err")
m3("stationary |error| p95", "stat_p95")
m3("moving |error| mean", "move_err")
m3("moving |error| p95", "move_p95")
m3("filter lag (moving)", "filt_lag")
m3("lag direction %", "lag_pct", "%")
print()

# ── Per-window detail for v3 ───────────────────────────────────────────
print("=" * 90)
print("=== v3 (CURRENT): PER-WINDOW DETAIL ===")
print("=" * 90)
print()
v3 = data[labels[-1]]
wins = find_windows(v3["cols"])
print(f"  {'Win':4s} {'Dur':>5s} {'SP p2p':>9s} {'Act p2p':>9s} {'|Err|':>9s} {'Bias':>9s} {'ErrStd':>9s} {'Sign%':>7s} {'VoltStd':>8s}")
for i, m in enumerate(v3["wm"]):
    s, e = wins[i] if i < len(wins) else (0, 0)
    dur = v3["cols"]["timestamp"][e-1] - v3["cols"]["timestamp"][s] if i < len(wins) else 0
    print(f"  {i:4d} {dur:4.1f}s {m['sp_p2p']:8.4f}° {m['act_p2p']:8.4f}° "
          f"{m['err_abs_mean']:8.4f}° {m['err_bias']:+8.4f}° {m['err_std']:8.4f}° "
          f"{m['err_sign_chg']:6.1f}% {m['volt_std']:7.4f}V")
print()

# ── Remaining issues diagnosis ─────────────────────────────────────────
print("=" * 90)
print("=== REMAINING ISSUES & RECOMMENDATIONS ===")
print("=" * 90)
print()

act_p2p = avg(labels[-1], "act_p2p")
sp_p2p = avg(labels[-1], "sp_p2p")
err_mean = avg(labels[-1], "err_abs_mean")
err_bias = avg(labels[-1], "err_bias")
err_std = avg(labels[-1], "err_std")
sign_chg = avg(labels[-1], "err_sign_chg")
move_err = data[labels[-1]]["mm"]["move_err"]
filt_lag = data[labels[-1]]["mm"]["filt_lag"]
lag_pct = data[labels[-1]]["mm"]["lag_pct"]

issues = []

if act_p2p > 0.5:
    amp = act_p2p / sp_p2p if sp_p2p > 0.001 else float('inf')
    if amp > 2.0:
        issues.append(f"MOTOR AMPLIFICATION: actual p2p ({act_p2p:.3f}°) is {amp:.1f}x setpoint p2p ({sp_p2p:.3f}°)\n"
                      f"     -> Increase kD (currently 1.0, try 2.0-3.0)")

if err_mean > 0.3:
    issues.append(f"STEADY-STATE ERROR: mean |error| = {err_mean:.3f}°, bias = {err_bias:+.3f}°\n"
                  f"     -> kI=0.5 may need more time to accumulate, or increase to kI=1.0")

if sign_chg > 30:
    issues.append(f"ERROR OSCILLATION: sign changes {sign_chg:.0f}% of cycles — motor is overshooting\n"
                  f"     -> kD may need to increase, or kI is causing integral windup")

if sign_chg < 5 and err_mean > 0.3:
    issues.append(f"NO SIGN CROSSING: error never crosses zero ({sign_chg:.1f}% sign changes)\n"
                  f"     -> kI=0.5 is not strong enough to overcome static friction\n"
                  f"     -> Try kI=1.0-2.0, or increase kS (currently 0.48V)")

if move_err > 2.0:
    issues.append(f"MOTION TRACKING: moving |error| = {move_err:.2f}°, lags {lag_pct:.0f}% of the time\n"
                  f"     -> Filter lag = {filt_lag:.2f}° avg. Consider SMOOTHING_ALPHA=0.7-0.8")

if filt_lag > 0.5:
    issues.append(f"FILTER LAG: {filt_lag:.2f}° avg during motion (alpha=0.60)\n"
                  f"     -> Increase toward 0.8, or add velocity feedforward to the filter")

if not issues:
    print("  No significant issues remaining. Performance looks good!")
else:
    for i, issue in enumerate(issues):
        print(f"  [{i+1}] {issue}")
        print()
