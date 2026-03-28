"""
v5 analysis: PositionVoltage + kI=1.0 + SyncCANcoder + alpha=0.80
Compare against v1 (original) and v4 (aggressive test).
"""
import csv
import numpy as np

FILES = {
    "v1_original":  "turret_diag_20260328_170907.csv",
    "v4_aggressive": "turret_diag_20260328_180101.csv",
    "v5_posVoltage": "turret_diag_20260328_181909.csv",
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

def full_analysis(cols):
    n = len(cols["timestamp"])
    dt = np.diff(cols["timestamp"])
    speed = np.sqrt(cols["vel_vx"]**2 + cols["vel_vy"]**2)
    omega = np.abs(cols["vel_omega"])
    t0 = cols["timestamp"][0]
    settled = cols["timestamp"] > (t0 + 2.0)
    stationary = settled & (speed < 0.05) & (omega < 0.05)
    moving = settled & ((speed >= 0.1) | (omega >= 0.1))
    err = cols["turret_error"]
    abs_err = np.abs(err)
    sp = cols["turret_setpoint"]
    act = cols["turret_actual"]
    sp_rate = np.zeros(n)
    sp_rate[1:] = np.diff(sp) / np.maximum(dt, 0.001)
    act_rate = np.zeros(n)
    act_rate[1:] = np.diff(act) / np.maximum(dt, 0.001)
    filt_lag = cols["raw_turret_angle"] - cols["filtered_turret_angle"]

    # Stationary window analysis
    wins = find_windows(cols)
    stat_errs = []
    win_details = []
    for s, e in wins:
        settle = min(50, (e - s) // 4)
        sl = slice(s + settle, e)
        if len(cols["timestamp"][sl]) < 20: continue
        w_err = err[sl]
        w_abs = np.abs(w_err)
        sc = np.sum(np.diff(np.sign(w_err)) != 0) / max(len(w_err)-1, 1) * 100
        stat_errs.extend(w_abs.tolist())
        win_details.append({
            "dur": cols["timestamp"][e-1] - cols["timestamp"][s],
            "err_mean": np.mean(w_abs),
            "bias": np.mean(w_err),
            "act_p2p": np.ptp(act[sl]),
            "sp_p2p": np.ptp(sp[sl]),
            "sign_chg": sc,
        })
    stat_errs = np.array(stat_errs) if stat_errs else np.array([0])

    # Omega buckets
    omega_analysis = {}
    for lo, hi in [(0, 0.1), (0.1, 0.5), (0.5, 1.0), (1.0, 2.0), (2.0, 99)]:
        mask = settled & (omega >= lo) & (omega < hi)
        cnt = np.sum(mask)
        if cnt < 5: continue
        omega_analysis[f"{lo}-{hi}"] = {
            "cnt": cnt,
            "err_mean": np.mean(abs_err[mask]),
            "err_p95": np.percentile(abs_err[mask], 95),
            "sp_rate": np.mean(np.abs(sp_rate[mask])),
            "act_rate": np.mean(np.abs(act_rate[mask])),
        }

    # Direction change analysis (integral windup check)
    sp_dir_change = np.zeros(n, dtype=bool)
    for i in range(2, n):
        if sp_rate[i] * sp_rate[i-1] < 0 and abs(sp_rate[i]) > 5:
            sp_dir_change[i] = True
    pre_err, post_err = [], []
    for i in np.where(sp_dir_change)[0]:
        if i > 10 and i < n - 10:
            pre_err.append(np.mean(abs_err[max(0,i-10):i]))
            post_err.append(np.mean(abs_err[i:min(n,i+10)]))

    return {
        "n": n,
        "duration": cols["timestamp"][-1] - cols["timestamp"][0],
        "n_stat": int(np.sum(stationary)),
        "n_move": int(np.sum(moving)),
        "stat_err_mean": np.mean(stat_errs),
        "stat_err_p95": np.percentile(stat_errs, 95) if len(stat_errs) > 1 else 0,
        "stat_err_max": np.max(stat_errs),
        "stat_bias_mean": np.mean([w["bias"] for w in win_details]) if win_details else 0,
        "stat_act_p2p": np.mean([w["act_p2p"] for w in win_details]) if win_details else 0,
        "stat_sign_chg": np.mean([w["sign_chg"] for w in win_details]) if win_details else 0,
        "move_err_mean": np.mean(abs_err[moving]) if np.sum(moving) > 0 else 0,
        "move_err_p95": np.percentile(abs_err[moving], 95) if np.sum(moving) > 0 else 0,
        "filt_lag_move": np.mean(np.abs(filt_lag[moving])) if np.sum(moving) > 0 else 0,
        "p99_sp_rate": np.percentile(np.abs(sp_rate[settled]), 99) if np.sum(settled) > 0 else 0,
        "p99_act_rate": np.percentile(np.abs(act_rate[settled]), 99) if np.sum(settled) > 0 else 0,
        "windup_pre": np.mean(pre_err) if pre_err else 0,
        "windup_post": np.mean(post_err) if post_err else 0,
        "omega_analysis": omega_analysis,
        "win_details": win_details,
        "volt_std_stat": np.std(cols["motor_voltage"][stationary]) if np.sum(stationary) > 0 else 0,
    }

# ── Analyze all ────────────────────────────────────────────────────────
results = {}
for label, path in FILES.items():
    cols = load(path)
    results[label] = full_analysis(cols)

v1 = results["v1_original"]
v4 = results["v4_aggressive"]
v5 = results["v5_posVoltage"]

# ── High-level comparison ──────────────────────────────────────────────
print("=" * 90)
print("=== OVERALL COMPARISON: v1 (original) vs v5 (current) ===")
print("=" * 90)
print()
print(f"  {'Metric':40s} {'v1 original':>14s} {'v5 current':>14s} {'Change':>10s}")
print(f"  {'-'*40} {'-'*14} {'-'*14} {'-'*10}")

def cmp(name, k, unit="°"):
    a, b = v1[k], v5[k]
    pct = (b - a) / a * 100 if a != 0 else 0
    tag = "BETTER" if b < a * 0.95 else ("WORSE" if b > a * 1.05 else "~same")
    print(f"  {name:40s} {a:12.3f}{unit} {b:12.3f}{unit} {pct:+7.0f}% {tag}")

cmp("Stationary |error| mean", "stat_err_mean")
cmp("Stationary |error| p95", "stat_err_p95")
cmp("Stationary actual p2p", "stat_act_p2p")
cmp("Moving |error| mean", "move_err_mean")
cmp("Moving |error| p95", "move_err_p95")
cmp("Filter lag (moving)", "filt_lag_move")
cmp("Voltage std (stationary)", "volt_std_stat", "V")
print()

# ── v4 vs v5 (both aggressive tests) ──────────────────────────────────
print("=" * 90)
print("=== AGGRESSIVE TEST: v4 (MotionMagic) vs v5 (PositionVoltage) ===")
print("=" * 90)
print()
print(f"  {'Metric':40s} {'v4 MMotion':>14s} {'v5 PosVolt':>14s} {'Change':>10s}")
print(f"  {'-'*40} {'-'*14} {'-'*14} {'-'*10}")

def cmp45(name, k, unit="°"):
    a, b = v4[k], v5[k]
    pct = (b - a) / a * 100 if a != 0 else 0
    tag = "BETTER" if b < a * 0.95 else ("WORSE" if b > a * 1.05 else "~same")
    print(f"  {name:40s} {a:12.3f}{unit} {b:12.3f}{unit} {pct:+7.0f}% {tag}")

cmp45("Stationary |error| mean", "stat_err_mean")
cmp45("Stationary |error| p95", "stat_err_p95")
cmp45("Stationary error bias", "stat_bias_mean")
cmp45("Stationary sign changes %", "stat_sign_chg", "%")
cmp45("Moving |error| mean", "move_err_mean")
cmp45("Moving |error| p95", "move_err_p95")
cmp45("Filter lag (moving)", "filt_lag_move")
cmp45("P99 setpoint rate", "p99_sp_rate", "°/s")
cmp45("P99 actual rate", "p99_act_rate", "°/s")
cmp45("Windup: error before dir change", "windup_pre")
cmp45("Windup: error after dir change", "windup_post")
print()

# ── Error by omega comparison ──────────────────────────────────────────
print("=" * 90)
print("=== ERROR BY ROTATION SPEED: v4 vs v5 ===")
print("=" * 90)
print()
print(f"  {'Omega':15s}  {'v4 |Err|':>10s} {'v5 |Err|':>10s} {'Change':>8s}  "
      f"{'v4 ActRate':>10s} {'v5 ActRate':>10s} {'Change':>8s}")
all_omega_keys = sorted(set(list(v4["omega_analysis"].keys()) + list(v5["omega_analysis"].keys())))
for k in all_omega_keys:
    o4 = v4["omega_analysis"].get(k)
    o5 = v5["omega_analysis"].get(k)
    if not o4 or not o5: continue
    err_pct = (o5["err_mean"] - o4["err_mean"]) / o4["err_mean"] * 100 if o4["err_mean"] > 0 else 0
    rate_pct = (o5["act_rate"] - o4["act_rate"]) / o4["act_rate"] * 100 if o4["act_rate"] > 0 else 0
    print(f"  {k:15s}  {o4['err_mean']:9.2f}° {o5['err_mean']:9.2f}° {err_pct:+7.0f}%  "
          f"{o4['act_rate']:9.1f}° {o5['act_rate']:9.1f}° {rate_pct:+7.0f}%")
print()

# ── v5 per-window detail ──────────────────────────────────────────────
print("=" * 90)
print("=== v5 PER-WINDOW STATIONARY DETAIL ===")
print("=" * 90)
print()
print(f"  {'Win':4s} {'Dur':>5s} {'SP p2p':>9s} {'Act p2p':>9s} {'|Err|':>9s} {'Bias':>9s} {'Sign%':>7s}")
for i, w in enumerate(v5["win_details"]):
    print(f"  {i:4d} {w['dur']:4.1f}s {w['sp_p2p']:8.4f}° {w['act_p2p']:8.4f}° "
          f"{w['err_mean']:8.4f}° {w['bias']:+8.4f}° {w['sign_chg']:6.1f}%")
print()

# ── Diagnosis ──────────────────────────────────────────────────────────
print("=" * 90)
print("=== DIAGNOSIS & NEXT STEPS ===")
print("=" * 90)
print()

print(f"Current config: kP=80, kI=1.0, kD=1.0, kS=0.48, alpha=0.80, SyncCANcoder, PositionVoltage")
print()

issues = []

# Stationary precision
if v5["stat_err_mean"] > 0.3:
    bias = abs(v5["stat_bias_mean"])
    if v5["stat_sign_chg"] < 5:
        issues.append((
            "STEADY-STATE OFFSET",
            v5["stat_err_mean"],
            f"|error| = {v5['stat_err_mean']:.3f}°, bias = {v5['stat_bias_mean']:+.3f}°, "
            f"sign changes = {v5['stat_sign_chg']:.1f}%\n"
            f"     Error never crosses zero — kI not overcoming static friction.\n"
            f"     Options:\n"
            f"       a) Increase kI to 2.0-3.0 (more aggressive integral)\n"
            f"       b) Increase kS from 0.48 to 0.6-0.8 (better friction compensation)\n"
            f"       c) Both — kS handles friction, kI handles remaining offset"
        ))
    else:
        issues.append((
            "STATIONARY OSCILLATION",
            v5["stat_err_mean"],
            f"|error| = {v5['stat_err_mean']:.3f}°, sign changes = {v5['stat_sign_chg']:.1f}%\n"
            f"     Error oscillates around setpoint — possible overshoot from kI or kP.\n"
            f"     -> Try increasing kD to 2.0 to damp oscillation"
        ))

# Integral windup
if v5["windup_post"] > v5["windup_pre"] * 2.0 and v5["windup_post"] > 5:
    issues.append((
        "INTEGRAL WINDUP",
        v5["windup_post"],
        f"Error jumps from {v5['windup_pre']:.1f}° to {v5['windup_post']:.1f}° after direction changes.\n"
        f"     Stored integral overshoots when setpoint reverses.\n"
        f"     -> Reduce kI to 0.5, rely more on kP+kD for tracking"
    ))

# Motion tracking
if v5["move_err_mean"] > 3.0:
    issues.append((
        "MOTION TRACKING LAG",
        v5["move_err_mean"],
        f"Moving |error| = {v5['move_err_mean']:.2f}° mean, {v5['move_err_p95']:.2f}° p95\n"
        f"     Filter lag = {v5['filt_lag_move']:.2f}°.\n"
        f"     P99 SP rate = {v5['p99_sp_rate']:.0f}°/s, actual = {v5['p99_act_rate']:.0f}°/s"
    ))

# Turret speed saturation
if v5["p99_act_rate"] < v5["p99_sp_rate"] * 0.6:
    issues.append((
        "TURRET SPEED SATURATION",
        v5["p99_sp_rate"],
        f"P99 actual rate ({v5['p99_act_rate']:.0f}°/s) << setpoint rate ({v5['p99_sp_rate']:.0f}°/s)\n"
        f"     During extreme spins, motor can't physically keep up.\n"
        f"     -> Check current limits (supply=20A may be too conservative)\n"
        f"     -> Or accept as physical limitation during extreme maneuvers"
    ))

if not issues:
    print("  Performance looks good! No significant issues remaining.")
else:
    issues.sort(key=lambda x: x[1], reverse=True)
    for i, (name, sev, desc) in enumerate(issues):
        print(f"  [{i+1}] {name}")
        print(f"     {desc}")
        print()
