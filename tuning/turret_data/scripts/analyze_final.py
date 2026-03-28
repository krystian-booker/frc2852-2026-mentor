"""Final comparison: v1 (original) vs v10 (final tune)"""
import csv, numpy as np

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
    sp_rate = np.zeros(n); sp_rate[1:] = np.diff(sp) / np.maximum(dt, 0.001)
    act_rate = np.zeros(n); act_rate[1:] = np.diff(act) / np.maximum(dt, 0.001)
    cur = cols["stator_current"]

    wins = find_windows(cols)
    stat_errs, win_details = [], []
    for s, e in wins:
        settle = min(50, (e - s) // 4)
        sl = slice(s + settle, e)
        if len(cols["timestamp"][sl]) < 20: continue
        w_err = err[sl]; w_abs = np.abs(w_err)
        sc = np.sum(np.diff(np.sign(w_err)) != 0) / max(len(w_err) - 1, 1) * 100
        stat_errs.extend(w_abs.tolist())
        win_details.append({"dur": cols["timestamp"][e-1] - cols["timestamp"][s],
            "err_mean": np.mean(w_abs), "bias": np.mean(w_err),
            "act_p2p": np.ptp(act[sl]), "sign_chg": sc})
    stat_errs = np.array(stat_errs) if stat_errs else np.array([0])

    omega_err = {}
    for lo, hi in [(0, 0.1), (0.1, 0.5), (0.5, 1.0), (1.0, 2.0), (2.0, 4.0), (4.0, 99)]:
        mask = settled & (omega >= lo) & (omega < hi)
        if np.sum(mask) < 5: continue
        omega_err[f"{lo}-{hi}"] = {"cnt": int(np.sum(mask)),
            "err": np.mean(abs_err[mask]), "p95": np.percentile(abs_err[mask], 95),
            "act_rate": np.mean(np.abs(act_rate[mask]))}

    sp_dir = np.zeros(n, dtype=bool)
    for i in range(2, n):
        if sp_rate[i] * sp_rate[i-1] < 0 and abs(sp_rate[i]) > 5: sp_dir[i] = True
    post = []
    for i in np.where(sp_dir)[0]:
        if i < n - 10: post.append(np.mean(abs_err[i:min(n, i+10)]))

    return {
        "stat_err": np.mean(stat_errs), "stat_p95": np.percentile(stat_errs, 95) if len(stat_errs) > 1 else 0,
        "stat_bias": np.mean([w["bias"] for w in win_details]) if win_details else 0,
        "stat_act_p2p": np.mean([w["act_p2p"] for w in win_details]) if win_details else 0,
        "stat_sign": np.mean([w["sign_chg"] for w in win_details]) if win_details else 0,
        "move_err": np.mean(abs_err[moving]) if np.sum(moving) > 0 else 0,
        "move_p95": np.percentile(abs_err[moving], 95) if np.sum(moving) > 0 else 0,
        "max_act_rate": np.max(np.abs(act_rate[settled])) if np.sum(settled) > 0 else 0,
        "windup_post": np.mean(post) if post else 0,
        "omega": omega_err, "wins": win_details,
    }

v1 = analyze(load("turret_diag_20260328_170907.csv"))
v10 = analyze(load("turret_diag_20260328_190541.csv"))

print("=" * 70)
print("  FINAL RESULTS: v1 (original) vs v10 (final)")
print("=" * 70)
print()
print("  v1: kP=80 kI=0   kD=0   kS=0.48 FusedCANcoder MotionMagic 20A")
print("  v10: kP=80 kI=1.0 kD=1.5 kS=1.20 SyncCANcoder  PosVoltage  60A")
print("       + filter removed")
print()
print(f"  {'Metric':35s} {'BEFORE':>10s} {'AFTER':>10s} {'Change':>10s}")
print(f"  {'-'*35} {'-'*10} {'-'*10} {'-'*10}")

def row(name, a, b, u="deg", lower_better=True):
    pct = (b - a) / a * 100 if a != 0 else 0
    good = (b < a * 0.9) if lower_better else (b > a * 1.1)
    bad = (b > a * 1.1) if lower_better else (b < a * 0.9)
    icon = "OK" if good else ("!!" if bad else "~~")
    s = "d" if u == "deg" else u
    print(f"  [{icon}] {name:33s} {a:8.3f}{s} {b:8.3f}{s}  {pct:+6.0f}%")

print()
print("  STATIONARY:")
row("  |error| mean", v1["stat_err"], v10["stat_err"])
row("  |error| p95", v1["stat_p95"], v10["stat_p95"])
row("  |bias|", abs(v1["stat_bias"]), abs(v10["stat_bias"]))
row("  actual p2p (jitter)", v1["stat_act_p2p"], v10["stat_act_p2p"])
print()
print("  MOTION:")
row("  |error| mean", v1["move_err"], v10["move_err"])
row("  |error| p95", v1["move_p95"], v10["move_p95"])
row("  max turret speed", v1["max_act_rate"], v10["max_act_rate"], "d/s", False)
print()
print("  STABILITY:")
row("  windup (post dir change)", v1["windup_post"], v10["windup_post"])

print()
print("=" * 70)
print("  ERROR BY ROTATION SPEED")
print("=" * 70)
print(f"  {'Omega':12s} {'BEFORE':>10s} {'AFTER':>10s} {'Change':>8s}")
for k in sorted(set(list(v1["omega"].keys()) + list(v10["omega"].keys()))):
    o1, o10 = v1["omega"].get(k), v10["omega"].get(k)
    if not o1 or not o10: continue
    pct = (o10["err"] - o1["err"]) / o1["err"] * 100 if o1["err"] > 0 else 0
    tag = "BETTER" if pct < -10 else ("WORSE" if pct > 10 else "~same")
    print(f"  {k:12s} {o1['err']:8.2f}d {o10['err']:8.2f}d  {pct:+6.0f}% {tag}")

print()
print("=" * 70)
print("  PER-WINDOW DETAIL (v10)")
print("=" * 70)
print(f"  {'Win':4s} {'Dur':>5s} {'|Err|':>9s} {'Bias':>9s} {'Act p2p':>9s} {'Sign%':>7s}")
for i, w in enumerate(v10["wins"]):
    print(f"  {i:4d} {w['dur']:4.1f}s {w['err_mean']:8.4f}d {w['bias']:+8.4f}d "
          f"{w['act_p2p']:8.4f}d {w['sign_chg']:6.1f}%")

print()
print("=" * 70)
print("  VERDICT")
print("=" * 70)
print()

all_good = True
if v10["stat_err"] < 0.5:
    print("  Stationary precision:  EXCELLENT (<0.5 deg)")
elif v10["stat_err"] < 1.0:
    print("  Stationary precision:  GOOD (<1.0 deg)")
    all_good = False
else:
    print(f"  Stationary precision:  NEEDS WORK ({v10['stat_err']:.2f} deg)")
    all_good = False

if v10["stat_act_p2p"] < 0.1:
    print("  Physical stability:    ROCK SOLID (0 jitter)")
else:
    print(f"  Physical stability:    {v10['stat_act_p2p']:.3f} deg p2p")

if v10["windup_post"] < 10:
    print("  Integral windup:       ELIMINATED")
elif v10["windup_post"] < 30:
    print("  Integral windup:       CONTROLLED")
else:
    print(f"  Integral windup:       PRESENT ({v10['windup_post']:.0f} deg)")
    all_good = False

# Check sign changes for kS balance
if v10["stat_sign"] > 20:
    print(f"  Friction compensation: OVERSHOOTING (sign_chg={v10['stat_sign']:.0f}%)")
    print("    -> kS=1.2 may be slightly high, try 1.1")
    all_good = False
elif v10["stat_sign"] > 5:
    print(f"  Friction compensation: WELL BALANCED (sign_chg={v10['stat_sign']:.0f}%)")
elif v10["stat_sign"] < 5 and v10["stat_err"] > 0.5:
    print(f"  Friction compensation: NEEDS MORE (sign_chg={v10['stat_sign']:.0f}%)")
    all_good = False
else:
    print("  Friction compensation: GOOD")

print()
if all_good:
    print("  No further tuning recommended. This is dialed in.")
else:
    print("  Minor tuning opportunities remain (see above).")
print()
