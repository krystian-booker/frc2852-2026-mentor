"""
Compare turret performance: before vs after kD + filter alpha changes.
"""
import csv
import numpy as np

FILES = {
    "BEFORE (kD=0, alpha=0.15)": "turret_diag_20260328_170907.csv",
    "AFTER  (kD=1, alpha=0.40)": "turret_diag_20260328_173048.csv",
}

def load(path):
    with open(path, "r") as f:
        reader = csv.DictReader(f)
        rows = list(reader)
    return {key: np.array([float(row[key]) for row in rows]) for key in rows[0]}

def find_stationary_windows(cols, min_samples=50):
    n = len(cols["timestamp"])
    speed = np.sqrt(cols["vel_vx"]**2 + cols["vel_vy"]**2)
    stat = (speed < 0.05) & (np.abs(cols["vel_omega"]) < 0.05)
    windows = []
    start = None
    for i in range(n):
        if stat[i] and start is None:
            start = i
        elif not stat[i] and start is not None:
            windows.append((start, i))
            start = None
    if start is not None:
        windows.append((start, n))
    return [(s, e) for s, e in windows if (e - s) >= min_samples]

def analyze_windows(cols, windows):
    """Analyze all stationary windows, return per-window metrics."""
    all_metrics = []
    for s, e in windows:
        settle = min(50, (e - s) // 4)
        sl = slice(s + settle, e)
        d = {k: cols[k][sl] for k in cols}
        if len(d["timestamp"]) < 20:
            continue

        abs_err = np.abs(d["turret_error"])
        all_metrics.append({
            "heading_std": np.std(d["pose_heading"]),
            "heading_p2p": np.ptp(d["pose_heading"]),
            "raw_std": np.std(d["raw_turret_angle"]),
            "raw_p2p": np.ptp(d["raw_turret_angle"]),
            "filt_std": np.std(d["filtered_turret_angle"]),
            "filt_p2p": np.ptp(d["filtered_turret_angle"]),
            "sp_std": np.std(d["turret_setpoint"]),
            "sp_p2p": np.ptp(d["turret_setpoint"]),
            "act_std": np.std(d["turret_actual"]),
            "act_p2p": np.ptp(d["turret_actual"]),
            "err_std": np.std(d["turret_error"]),
            "err_p2p": np.ptp(d["turret_error"]),
            "err_mean": np.mean(d["turret_error"]),
            "err_abs_mean": np.mean(abs_err),
            "err_abs_p95": np.percentile(abs_err, 95),
            "volt_std": np.std(d["motor_voltage"]),
            "cur_std": np.std(d["stator_current"]),
            "err_sign_changes_pct": np.sum(np.diff(np.sign(d["turret_error"])) != 0) / max(len(d["turret_error"]) - 1, 1) * 100,
        })
    return all_metrics

def analyze_motion(cols):
    """Analyze motion tracking performance."""
    n = len(cols["timestamp"])
    speed = np.sqrt(cols["vel_vx"]**2 + cols["vel_vy"]**2)
    omega = np.abs(cols["vel_omega"])

    t0 = cols["timestamp"][0]
    settled = cols["timestamp"] > (t0 + 2.0)
    stationary = settled & (speed < 0.05) & (omega < 0.05)
    moving = settled & ((speed >= 0.1) | (omega >= 0.1))

    abs_err = np.abs(cols["turret_error"])

    # Filter lag
    filter_lag = cols["raw_turret_angle"] - cols["filtered_turret_angle"]

    # Lag direction
    sp = cols["turret_setpoint"]
    err = cols["turret_error"]
    m_idx = np.where(moving)[0]
    lag_count = 0
    total_dir = 0
    for i in m_idx:
        if i == 0:
            continue
        sp_change = sp[i] - sp[i-1]
        if abs(sp_change) < 0.001:
            continue
        total_dir += 1
        if np.sign(sp_change) == np.sign(err[i]):
            lag_count += 1

    return {
        "stat_err_mean": np.mean(abs_err[stationary]) if np.sum(stationary) > 0 else 0,
        "stat_err_p95": np.percentile(abs_err[stationary], 95) if np.sum(stationary) > 0 else 0,
        "move_err_mean": np.mean(abs_err[moving]) if np.sum(moving) > 0 else 0,
        "move_err_p95": np.percentile(abs_err[moving], 95) if np.sum(moving) > 0 else 0,
        "filter_lag_move_mean": np.mean(np.abs(filter_lag[moving])) if np.sum(moving) > 0 else 0,
        "filter_lag_move_p95": np.percentile(np.abs(filter_lag[moving]), 95) if np.sum(moving) > 0 else 0,
        "lag_pct": 100 * lag_count / max(total_dir, 1),
        "n_stat": int(np.sum(stationary)),
        "n_move": int(np.sum(moving)),
    }

# ── Run analysis on both files ─────────────────────────────────────────
all_data = {}
for label, path in FILES.items():
    cols = load(path)
    windows = find_stationary_windows(cols)
    metrics = analyze_windows(cols, windows)
    motion = analyze_motion(cols)
    all_data[label] = {
        "cols": cols,
        "windows": windows,
        "metrics": metrics,
        "motion": motion,
    }

labels = list(FILES.keys())
L1, L2 = labels[0], labels[1]

def avg_metric(data, key):
    return np.mean([m[key] for m in data["metrics"]])

# ── Side-by-side comparison ────────────────────────────────────────────
print("=" * 78)
print("=== BEFORE vs AFTER COMPARISON ===")
print("=" * 78)
print()

print(f"  {'Metric':40s} {'BEFORE':>12s} {'AFTER':>12s} {'Change':>12s}")
print(f"  {'-'*40} {'-'*12} {'-'*12} {'-'*12}")

def row(name, key, unit="deg", lower_is_better=True):
    v1 = avg_metric(all_data[L1], key)
    v2 = avg_metric(all_data[L2], key)
    delta = v2 - v1
    pct = (delta / v1 * 100) if v1 != 0 else 0
    if lower_is_better:
        indicator = "BETTER" if delta < -0.001 else ("WORSE" if delta > 0.001 else "same")
    else:
        indicator = "BETTER" if delta > 0.001 else ("WORSE" if delta < -0.001 else "same")
    print(f"  {name:40s} {v1:10.4f} {unit} {v2:10.4f} {unit} {pct:+7.1f}% {indicator}")

print("\n  STATIONARY — Pose Estimation:")
row("heading std", "heading_std")
row("heading p2p", "heading_p2p")

print("\n  STATIONARY — Aiming Calculator:")
row("raw_angle std", "raw_std")
row("raw_angle p2p", "raw_p2p")
row("filtered_angle std", "filt_std")
row("filtered_angle p2p", "filt_p2p")

print("\n  STATIONARY — Motor Control:")
row("setpoint std", "sp_std")
row("setpoint p2p", "sp_p2p")
row("actual std", "act_std")
row("actual p2p", "act_p2p")
row("error std", "err_std")
row("error |mean|", "err_abs_mean")
row("error p95", "err_abs_p95")
row("error sign changes %", "err_sign_changes_pct")
row("motor voltage std", "volt_std", unit="V")

print()

# ── Motion comparison ──────────────────────────────────────────────────
print("=" * 78)
print("=== MOTION TRACKING COMPARISON ===")
print("=" * 78)
print()

m1 = all_data[L1]["motion"]
m2 = all_data[L2]["motion"]

def mrow(name, key, unit="deg", lower_is_better=True):
    v1 = m1[key]
    v2 = m2[key]
    delta = v2 - v1
    pct = (delta / v1 * 100) if v1 != 0 else 0
    if lower_is_better:
        indicator = "BETTER" if delta < -0.001 else ("WORSE" if delta > 0.001 else "same")
    else:
        indicator = "BETTER" if delta > 0.001 else ("WORSE" if delta < -0.001 else "same")
    print(f"  {name:40s} {v1:10.4f} {unit} {v2:10.4f} {unit} {pct:+7.1f}% {indicator}")

print(f"  {'':40s} {'BEFORE':>12s} {'AFTER':>12s} {'Change':>12s}")
print(f"  {'-'*40} {'-'*12} {'-'*12} {'-'*12}")

mrow("stationary |error| mean", "stat_err_mean")
mrow("stationary |error| p95", "stat_err_p95")
mrow("moving |error| mean", "move_err_mean")
mrow("moving |error| p95", "move_err_p95")
mrow("filter lag (moving) mean", "filter_lag_move_mean")
mrow("filter lag (moving) p95", "filter_lag_move_p95")
mrow("lag direction %", "lag_pct", unit="%")
print()

# ── Per-window detail for AFTER ────────────────────────────────────────
print("=" * 78)
print("=== AFTER: PER-WINDOW DETAIL ===")
print("=" * 78)
print()

after = all_data[L2]
print(f"  {'Window':8s} {'Duration':>8s} {'Heading':>10s} {'SP p2p':>10s} {'Act p2p':>10s} {'|Err| mean':>10s} {'Err bias':>10s} {'Amplif':>8s}")
for i, (s, e) in enumerate(after["windows"]):
    if i >= len(after["metrics"]):
        break
    m = after["metrics"][i]
    dur = after["cols"]["timestamp"][e-1] - after["cols"]["timestamp"][s]
    amp = m["act_p2p"] / m["sp_p2p"] if m["sp_p2p"] > 0.001 else 0
    print(f"  {i:8d} {dur:7.1f}s {m['heading_p2p']:9.4f}° {m['sp_p2p']:9.4f}° {m['act_p2p']:9.4f}° {m['err_abs_mean']:9.4f}° {m['err_mean']:+9.4f}° {amp:7.1f}x")
print()

# ── Remaining issues ───────────────────────────────────────────────────
print("=" * 78)
print("=== REMAINING ISSUES & NEXT STEPS ===")
print("=" * 78)
print()

# Check stationary jitter
act_p2p_after = avg_metric(all_data[L2], "act_p2p")
act_p2p_before = avg_metric(all_data[L1], "act_p2p")
sp_p2p_after = avg_metric(all_data[L2], "sp_p2p")
err_mean_after = avg_metric(all_data[L2], "err_abs_mean")
err_bias_after = avg_metric(all_data[L2], "err_mean")
motor_amp = act_p2p_after / sp_p2p_after if sp_p2p_after > 0.001 else 0

if act_p2p_after > 0.5:
    print(f"  STATIONARY JITTER still present: actual p2p = {act_p2p_after:.3f} deg")
    if motor_amp > 2.0:
        print(f"    Motor still amplifies setpoint by {motor_amp:.1f}x -> kD may need to increase")
        print(f"    Try kD = 2.0-3.0")
    if sp_p2p_after > 0.3:
        print(f"    Setpoint still noisy ({sp_p2p_after:.3f} deg p2p) -> filter alpha may need to decrease")
    print()

if abs(err_bias_after) > 0.5:
    print(f"  STEADY-STATE BIAS: mean error = {err_bias_after:+.3f} deg")
    print(f"    Motor consistently undershoots by ~{abs(err_bias_after):.1f} deg")
    print(f"    -> Add small kI (try 0.5) to eliminate DC offset")
    print()

move_err_after = m2["move_err_mean"]
move_err_before = m1["move_err_mean"]
if move_err_after > 2.0:
    print(f"  MOTION TRACKING: moving error = {move_err_after:.2f} deg (was {move_err_before:.2f})")
    filt_lag = m2["filter_lag_move_mean"]
    if filt_lag > 1.0:
        print(f"    Filter lag during motion: {filt_lag:.2f} deg avg")
        print(f"    -> SMOOTHING_ALPHA=0.40 may still be too low for fast motion")
        print(f"    -> Consider alpha=0.6 or adding velocity feedforward to the filter")
    lag_pct = m2["lag_pct"]
    if lag_pct > 60:
        print(f"    Turret still lags {lag_pct:.0f}% of the time during motion")
    print()

sign_chg = avg_metric(all_data[L2], "err_sign_changes_pct")
if sign_chg > 30:
    print(f"  PID OSCILLATION: error changes sign {sign_chg:.0f}% of cycles")
    print(f"    kD may be too high, causing overshoot in the other direction")
    print(f"    -> Try reducing kD to 0.5")
    print()

if act_p2p_after < 0.3 and move_err_after < 1.5:
    print("  Both stationary and motion performance look good!")
    print("  No further tuning recommended at this time.")
print()
