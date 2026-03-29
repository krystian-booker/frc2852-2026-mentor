"""v8: kS=1.0, 60A current limits, kI=0.5, kD=1.5"""
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
    filt_lag = cols["raw_turret_angle"] - cols["filtered_turret_angle"]
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
            "sp_rate": np.mean(np.abs(sp_rate[mask])),
            "act_rate": np.mean(np.abs(act_rate[mask])),
            "cur": np.mean(np.abs(cur[mask]))}

    sp_dir = np.zeros(n, dtype=bool)
    for i in range(2, n):
        if sp_rate[i] * sp_rate[i-1] < 0 and abs(sp_rate[i]) > 5: sp_dir[i] = True
    pre, post = [], []
    for i in np.where(sp_dir)[0]:
        if 10 < i < n - 10:
            pre.append(np.mean(abs_err[max(0, i-10):i]))
            post.append(np.mean(abs_err[i:min(n, i+10)]))

    return {
        "stat_err": np.mean(stat_errs), "stat_p95": np.percentile(stat_errs, 95) if len(stat_errs) > 1 else 0,
        "stat_bias": np.mean([w["bias"] for w in win_details]) if win_details else 0,
        "stat_act_p2p": np.mean([w["act_p2p"] for w in win_details]) if win_details else 0,
        "stat_sign": np.mean([w["sign_chg"] for w in win_details]) if win_details else 0,
        "move_err": np.mean(abs_err[moving]) if np.sum(moving) > 0 else 0,
        "move_p95": np.percentile(abs_err[moving], 95) if np.sum(moving) > 0 else 0,
        "filt_lag": np.mean(np.abs(filt_lag[moving])) if np.sum(moving) > 0 else 0,
        "p99_act": np.percentile(np.abs(act_rate[settled]), 99) if np.sum(settled) > 0 else 0,
        "max_act_rate": np.max(np.abs(act_rate[settled])) if np.sum(settled) > 0 else 0,
        "windup_pre": np.mean(pre) if pre else 0, "windup_post": np.mean(post) if post else 0,
        "omega": omega_err, "wins": win_details,
        "max_current": np.max(np.abs(cur[settled])) if np.sum(settled) > 0 else 0,
        "p99_current": np.percentile(np.abs(cur[settled]), 99) if np.sum(settled) > 0 else 0,
    }

v1 = analyze(load("turret_diag_20260328_170907.csv"))
v7 = analyze(load("turret_diag_20260328_183800.csv"))
v8 = analyze(load("turret_diag_20260328_184621.csv"))

print("=" * 90)
print("=== v1 (original) vs v7 (last) vs v8 (kS=1.0, 60A limits) ===")
print("=" * 90)
print()
print(f"  {'Metric':35s} {'v1 original':>13s} {'v7 last':>13s} {'v8 current':>13s} {'v1->v8':>8s}")
print(f"  {'-'*35} {'-'*13} {'-'*13} {'-'*13} {'-'*8}")

def row(name, k, u="deg"):
    a, b, c = v1[k], v7[k], v8[k]
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
print()
print("  MOTION:")
row("  |error| mean", "move_err")
row("  |error| p95", "move_p95")
row("  filter lag", "filt_lag")
row("  P99 actual rate", "p99_act", "d/s")
row("  Max actual rate", "max_act_rate", "d/s")
print()
print("  WINDUP:")
row("  err after dir change", "windup_post")
print()
print("  CURRENT DRAW:")
print(f"  Max stator current:  v7={v7['max_current']:.1f}A  v8={v8['max_current']:.1f}A")
print(f"  P99 stator current:  v7={v7['p99_current']:.1f}A  v8={v8['p99_current']:.1f}A")

print()
print("=" * 90)
print("=== ERROR BY OMEGA: v7 vs v8 (+ current & rate) ===")
print("=" * 90)
print(f"  {'Omega':12s} {'v7 |Err|':>9s} {'v8 |Err|':>9s} {'Chg':>7s}  "
      f"{'v7 ActRt':>9s} {'v8 ActRt':>9s} {'Chg':>7s}  {'v8 Amps':>8s}")
for k in sorted(set(list(v7["omega"].keys()) + list(v8["omega"].keys()))):
    o7, o8 = v7["omega"].get(k), v8["omega"].get(k)
    if not o7 or not o8: continue
    ep = (o8["err"] - o7["err"]) / o7["err"] * 100 if o7["err"] > 0 else 0
    rp = (o8["act_rate"] - o7["act_rate"]) / o7["act_rate"] * 100 if o7["act_rate"] > 0 else 0
    print(f"  {k:12s} {o7['err']:8.2f}d {o8['err']:8.2f}d {ep:+6.0f}%  "
          f"{o7['act_rate']:8.1f}d {o8['act_rate']:8.1f}d {rp:+6.0f}%  {o8['cur']:7.1f}A")

print()
print("=" * 90)
print("=== v8 PER-WINDOW DETAIL ===")
print("=" * 90)
print(f"  {'Win':4s} {'Dur':>5s} {'|Err|':>9s} {'Bias':>9s} {'Act p2p':>9s} {'Sign%':>7s}")
for i, w in enumerate(v8["wins"]):
    print(f"  {i:4d} {w['dur']:4.1f}s {w['err_mean']:8.4f}d {w['bias']:+8.4f}d "
          f"{w['act_p2p']:8.4f}d {w['sign_chg']:6.1f}%")

# Fast spin deep dive
print()
print("=" * 90)
print("=== FAST SPIN DEEP DIVE ===")
print("=" * 90)
cols = load("turret_diag_20260328_184621.csv")
n = len(cols["timestamp"])
dt = np.diff(cols["timestamp"])
omega = np.abs(cols["vel_omega"])
err = cols["turret_error"]
sp = cols["turret_setpoint"]
act = cols["turret_actual"]
sp_rate = np.zeros(n); sp_rate[1:] = np.diff(sp) / np.maximum(dt, 0.001)
act_rate = np.zeros(n); act_rate[1:] = np.diff(act) / np.maximum(dt, 0.001)
cur = cols["stator_current"]
settled = cols["timestamp"] > (cols["timestamp"][0] + 2.0)

fast = settled & (omega > 2.0)
if np.sum(fast) > 5:
    print(f"  Samples at omega > 2 rad/s: {np.sum(fast)}")
    print(f"  Mean robot rotation: {np.mean(omega[fast]) * 180 / np.pi:.0f} deg/s")
    print(f"  Mean SP rate demanded: {np.mean(np.abs(sp_rate[fast])):.0f} deg/s")
    print(f"  Mean actual turret rate: {np.mean(np.abs(act_rate[fast])):.0f} deg/s")
    deficit = np.mean(np.abs(sp_rate[fast])) - np.mean(np.abs(act_rate[fast]))
    print(f"  Rate deficit: {deficit:.0f} deg/s")
    print(f"  Mean |error|: {np.mean(np.abs(err[fast])):.1f} deg")
    print(f"  Mean stator current: {np.mean(np.abs(cur[fast])):.1f} A")
    print(f"  Max stator current: {np.max(np.abs(cur[fast])):.1f} A")
    print()
    # Kraken X44 ~6000RPM / 50:1 = 120RPS / 50 = 2.4RPS = 864 deg/s
    max_theoretical = 864.0
    max_observed = np.max(np.abs(act_rate[settled]))
    print(f"  Theoretical max turret speed (Kraken 6000RPM / 50:1): ~{max_theoretical:.0f} deg/s")
    print(f"  Max observed actual rate: {max_observed:.0f} deg/s")
    print(f"  -> Turret reaching {max_observed / max_theoretical * 100:.0f}% of theoretical max")
    print()
    at_limit = fast & (np.abs(cur) > 55)
    print(f"  Samples near current limit (>55A): {np.sum(at_limit)} ({100 * np.sum(at_limit) / max(np.sum(fast), 1):.0f}%)")

    # Is the SP rate exceeding what the turret can physically do?
    print()
    print("  SP rate vs physical limit during fast spins:")
    for threshold in [200, 400, 600, 800]:
        exceeds = fast & (np.abs(sp_rate) > threshold)
        if np.sum(exceeds) > 0:
            print(f"    SP rate > {threshold} deg/s: {np.sum(exceeds)} samples, "
                  f"mean |err|={np.mean(np.abs(err[exceeds])):.1f} deg, "
                  f"mean act_rate={np.mean(np.abs(act_rate[exceeds])):.0f} deg/s")
else:
    print("  No fast spin samples found (omega > 2 rad/s)")

print()
print("=" * 90)
print("=== DIAGNOSIS ===")
print("=" * 90)
print()
print("Current: kP=80, kI=0.5, kD=1.5, kS=1.0, alpha=0.80, SyncCANcoder, PositionVoltage")
print(f"Current limits: supply=60A, stator=60A")
print()

if v8["stat_sign"] < 5 and v8["stat_err"] > 0.3:
    print(f"  [STEADY-STATE] |err|={v8['stat_err']:.3f}d, bias={v8['stat_bias']:+.3f}d, sign_chg={v8['stat_sign']:.1f}%")
    print(f"    Still one-sided. kS=1.0 may still not be enough, or kI=0.5 too slow.")
elif v8["stat_sign"] > 15:
    print(f"  [STEADY-STATE] |err|={v8['stat_err']:.3f}d, sign_chg={v8['stat_sign']:.1f}%")
    print(f"    Crossing zero — friction is compensated. Fine-tune for tighter settling.")
elif v8["stat_err"] < 0.3:
    print(f"  [STEADY-STATE] |err|={v8['stat_err']:.3f}d — excellent!")

if np.sum(fast) > 5:
    obs_max = np.max(np.abs(act_rate[settled]))
    if obs_max < 700:
        print(f"  [SPEED] Max turret rate {obs_max:.0f} deg/s — still current-limited or gear-limited")
    else:
        print(f"  [SPEED] Max turret rate {obs_max:.0f} deg/s — approaching mechanical limits")
print()
