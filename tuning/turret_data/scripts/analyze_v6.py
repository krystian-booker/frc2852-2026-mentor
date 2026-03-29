"""
v6 analysis: kS=0.70, kI=0.5, kD=2.0
Compare v1 (original), v5 (last test), v6 (current)
"""
import csv
import numpy as np

FILES = {
    "v1_original":  "turret_diag_20260328_170907.csv",
    "v5_posVolt":   "turret_diag_20260328_181909.csv",
    "v6_kS_tune":   "turret_diag_20260328_182848.csv",
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

def analyze(cols):
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

    wins = find_windows(cols)
    stat_errs, win_details = [], []
    for s, e in wins:
        settle = min(50, (e - s) // 4)
        sl = slice(s + settle, e)
        if len(cols["timestamp"][sl]) < 20:
            continue
        w_err = err[sl]
        w_abs = np.abs(w_err)
        sc = np.sum(np.diff(np.sign(w_err)) != 0) / max(len(w_err) - 1, 1) * 100
        stat_errs.extend(w_abs.tolist())
        win_details.append({
            "dur": cols["timestamp"][e - 1] - cols["timestamp"][s],
            "err_mean": np.mean(w_abs), "bias": np.mean(w_err),
            "act_p2p": np.ptp(act[sl]), "sp_p2p": np.ptp(sp[sl]), "sign_chg": sc,
        })
    stat_errs = np.array(stat_errs) if stat_errs else np.array([0])

    omega_err = {}
    for lo, hi in [(0, 0.1), (0.1, 0.5), (0.5, 1.0), (1.0, 2.0), (2.0, 99)]:
        mask = settled & (omega >= lo) & (omega < hi)
        if np.sum(mask) < 5:
            continue
        omega_err[f"{lo}-{hi}"] = {
            "cnt": int(np.sum(mask)),
            "err": np.mean(abs_err[mask]),
            "p95": np.percentile(abs_err[mask], 95),
            "sp_rate": np.mean(np.abs(sp_rate[mask])),
            "act_rate": np.mean(np.abs(act_rate[mask])),
        }

    sp_dir = np.zeros(n, dtype=bool)
    for i in range(2, n):
        if sp_rate[i] * sp_rate[i - 1] < 0 and abs(sp_rate[i]) > 5:
            sp_dir[i] = True
    pre, post = [], []
    for i in np.where(sp_dir)[0]:
        if 10 < i < n - 10:
            pre.append(np.mean(abs_err[max(0, i - 10):i]))
            post.append(np.mean(abs_err[i:min(n, i + 10)]))

    return {
        "stat_err": np.mean(stat_errs),
        "stat_p95": np.percentile(stat_errs, 95) if len(stat_errs) > 1 else 0,
        "stat_bias": np.mean([w["bias"] for w in win_details]) if win_details else 0,
        "stat_act_p2p": np.mean([w["act_p2p"] for w in win_details]) if win_details else 0,
        "stat_sign": np.mean([w["sign_chg"] for w in win_details]) if win_details else 0,
        "move_err": np.mean(abs_err[moving]) if np.sum(moving) > 0 else 0,
        "move_p95": np.percentile(abs_err[moving], 95) if np.sum(moving) > 0 else 0,
        "filt_lag": np.mean(np.abs(filt_lag[moving])) if np.sum(moving) > 0 else 0,
        "p99_sp": np.percentile(np.abs(sp_rate[settled]), 99) if np.sum(settled) > 0 else 0,
        "p99_act": np.percentile(np.abs(act_rate[settled]), 99) if np.sum(settled) > 0 else 0,
        "windup_pre": np.mean(pre) if pre else 0,
        "windup_post": np.mean(post) if post else 0,
        "omega": omega_err,
        "wins": win_details,
        "volt_std": np.std(cols["motor_voltage"][stationary]) if np.sum(stationary) > 0 else 0,
    }

R = {k: analyze(load(p)) for k, p in FILES.items()}
v1, v5, v6 = R["v1_original"], R["v5_posVolt"], R["v6_kS_tune"]

print("=" * 90)
print("=== v1 (original) vs v5 (last test) vs v6 (current: kS=0.70 kI=0.5 kD=2.0) ===")
print("=" * 90)
print()
print(f"  {'Metric':35s} {'v1 original':>13s} {'v5 last':>13s} {'v6 current':>13s} {'v1->v6':>8s}")
print(f"  {'-' * 35} {'-' * 13} {'-' * 13} {'-' * 13} {'-' * 8}")

def row(name, k, u="deg"):
    a, b, c = v1[k], v5[k], v6[k]
    pct = (c - a) / a * 100 if a != 0 else 0
    tag = "BETTER" if c < a * 0.9 else ("WORSE" if c > a * 1.1 else "~same")
    s = "d" if u == "deg" else u[0]
    print(f"  {name:35s} {a:11.3f}{s} {b:11.3f}{s} {c:11.3f}{s} {pct:+6.0f}% {tag}")

print("  STATIONARY:")
row("  |error| mean", "stat_err")
row("  |error| p95", "stat_p95")
row("  error bias", "stat_bias")
row("  actual p2p", "stat_act_p2p")
row("  sign changes %", "stat_sign", "%")
row("  voltage std", "volt_std", "V")
print()
print("  MOTION:")
row("  |error| mean", "move_err")
row("  |error| p95", "move_p95")
row("  filter lag", "filt_lag")
row("  P99 setpoint rate", "p99_sp", "d/s")
row("  P99 actual rate", "p99_act", "d/s")
print()
print("  INTEGRAL WINDUP:")
row("  err before dir change", "windup_pre")
row("  err after dir change", "windup_post")

print()
print("=" * 90)
print("=== ERROR BY OMEGA: v5 vs v6 ===")
print("=" * 90)
print(f"  {'Omega':12s} {'v5 |Err|':>9s} {'v6 |Err|':>9s} {'Chg':>7s}  "
      f"{'v5 ActRate':>10s} {'v6 ActRate':>10s} {'Chg':>7s}")
for k in sorted(set(list(v5["omega"].keys()) + list(v6["omega"].keys()))):
    o5, o6 = v5["omega"].get(k), v6["omega"].get(k)
    if not o5 or not o6:
        continue
    ep = (o6["err"] - o5["err"]) / o5["err"] * 100 if o5["err"] > 0 else 0
    rp = (o6["act_rate"] - o5["act_rate"]) / o5["act_rate"] * 100 if o5["act_rate"] > 0 else 0
    print(f"  {k:12s} {o5['err']:8.2f}d {o6['err']:8.2f}d {ep:+6.0f}%  "
          f"{o5['act_rate']:9.1f}d {o6['act_rate']:9.1f}d {rp:+6.0f}%")

print()
print("=" * 90)
print("=== v6 PER-WINDOW DETAIL ===")
print("=" * 90)
print(f"  {'Win':4s} {'Dur':>5s} {'|Err|':>9s} {'Bias':>9s} {'Act p2p':>9s} {'Sign%':>7s}")
for i, w in enumerate(v6["wins"]):
    print(f"  {i:4d} {w['dur']:4.1f}s {w['err_mean']:8.4f}d {w['bias']:+8.4f}d "
          f"{w['act_p2p']:8.4f}d {w['sign_chg']:6.1f}%")

print()
print("=" * 90)
print("=== DIAGNOSIS ===")
print("=" * 90)
print()
print(f"Current: kP=80, kI=0.5, kD=2.0, kS=0.70, alpha=0.80, SyncCANcoder, PositionVoltage")
print()

if v6["stat_err"] > 0.3:
    if v6["stat_sign"] < 5:
        print(f"  [STEADY-STATE] |err|={v6['stat_err']:.3f}d, bias={v6['stat_bias']:+.3f}d, "
              f"sign_chg={v6['stat_sign']:.1f}%")
        print(f"    Error still one-sided. kS=0.70 not fully compensating friction.")
        print(f"    -> Try kS=0.90 and/or increase kI to 1.0")
    else:
        print(f"  [STATIONARY] |err|={v6['stat_err']:.3f}d, sign_chg={v6['stat_sign']:.1f}%")
        print(f"    Error is crossing zero — good. May need fine-tuning.")
elif v6["stat_err"] <= 0.3:
    print(f"  [STATIONARY] |err|={v6['stat_err']:.3f}d - looking good!")

print()
if v6["windup_post"] > v6["windup_pre"] * 1.5 and v6["windup_post"] > 5:
    ratio = v6["windup_post"] / max(v6["windup_pre"], 0.01)
    print(f"  [WINDUP] Before: {v6['windup_pre']:.1f}d -> After: {v6['windup_post']:.1f}d ({ratio:.1f}x spike)")
    print(f"    Consider software iZone: only accumulate kI when |error| < 5 degrees")
elif v6["windup_post"] > 5:
    print(f"  [WINDUP] Reduced but still present: {v6['windup_post']:.1f}d after dir changes")
else:
    print(f"  [WINDUP] Under control ({v6['windup_post']:.1f}d after dir changes)")

print()
if v6["move_err"] > 3:
    print(f"  [MOTION] |err|={v6['move_err']:.2f}d, filter lag={v6['filt_lag']:.2f}d")
    print(f"    Large errors dominated by extreme spin episodes.")
else:
    print(f"  [MOTION] |err|={v6['move_err']:.2f}d - acceptable")
print()
