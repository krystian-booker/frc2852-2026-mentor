"""v9: kS=1.2, alpha=1.0 (no filter), kI=0.5, kD=1.5, 60A limits"""
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
    }

v1 = analyze(load("turret_diag_20260328_170907.csv"))
v8 = analyze(load("turret_diag_20260328_184621.csv"))
v9 = analyze(load("turret_diag_20260328_185638.csv"))

# ── Main comparison ────────────────────────────────────────────────────
print("=" * 90)
print("=== v1 (original) vs v8 (last) vs v9 (kS=1.2, no filter) ===")
print("=" * 90)
print()
print(f"  {'Metric':35s} {'v1 original':>13s} {'v8 last':>13s} {'v9 current':>13s} {'v1->v9':>8s}")
print(f"  {'-'*35} {'-'*13} {'-'*13} {'-'*13} {'-'*8}")

def row(name, k, u="deg"):
    a, b, c = v1[k], v8[k], v9[k]
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

# ── Omega comparison v8 vs v9 ─────────────────────────────────────────
print()
print("=" * 90)
print("=== ERROR BY OMEGA: v8 (last) vs v9 (current) ===")
print("=" * 90)
print(f"  {'Omega':12s} {'v8 |Err|':>9s} {'v9 |Err|':>9s} {'Chg':>7s}  "
      f"{'v8 ActRt':>9s} {'v9 ActRt':>9s} {'Chg':>7s}  {'v9 Amps':>8s}")
for k in sorted(set(list(v8["omega"].keys()) + list(v9["omega"].keys()))):
    o8, o9 = v8["omega"].get(k), v9["omega"].get(k)
    if not o8 or not o9: continue
    ep = (o9["err"] - o8["err"]) / o8["err"] * 100 if o8["err"] > 0 else 0
    rp = (o9["act_rate"] - o8["act_rate"]) / o8["act_rate"] * 100 if o8["act_rate"] > 0 else 0
    print(f"  {k:12s} {o8['err']:8.2f}d {o9['err']:8.2f}d {ep:+6.0f}%  "
          f"{o8['act_rate']:8.1f}d {o9['act_rate']:8.1f}d {rp:+6.0f}%  {o9['cur']:7.1f}A")

# ── v9 per-window ─────────────────────────────────────────────────────
print()
print("=" * 90)
print("=== v9 PER-WINDOW DETAIL ===")
print("=" * 90)
print(f"  {'Win':4s} {'Dur':>5s} {'|Err|':>9s} {'Bias':>9s} {'Act p2p':>9s} {'Sign%':>7s}")
for i, w in enumerate(v9["wins"]):
    print(f"  {i:4d} {w['dur']:4.1f}s {w['err_mean']:8.4f}d {w['bias']:+8.4f}d "
          f"{w['act_p2p']:8.4f}d {w['sign_chg']:6.1f}%")

# ── Final scorecard ───────────────────────────────────────────────────
print()
print("=" * 90)
print("=== FULL JOURNEY: v1 -> v9 SCORECARD ===")
print("=" * 90)
print()
print("  Config evolution:")
print("    v1: kP=80 kI=0   kD=0   kS=0.48 alpha=0.15 FusedCANcoder MotionMagic  20A")
print("    v9: kP=80 kI=0.5 kD=1.5 kS=1.20 alpha=1.00 SyncCANcoder  PosVoltage   60A")
print()

metrics = [
    ("Stationary |error| mean", "stat_err", True),
    ("Stationary actual p2p", "stat_act_p2p", True),
    ("Filter lag (motion)", "filt_lag", True),
    ("Windup (post dir change)", "windup_post", True),
    ("Max turret speed", "max_act_rate", False),
]
for name, k, lower_better in metrics:
    before, after = v1[k], v9[k]
    pct = (after - before) / before * 100 if before != 0 else 0
    good = (after < before * 0.9) if lower_better else (after > before * 1.1)
    bad = (after > before * 1.1) if lower_better else (after < before * 0.9)
    icon = "OK" if good else ("!!" if bad else "~~")
    print(f"  [{icon}] {name:35s} {before:8.3f} -> {after:8.3f} deg  ({pct:+.0f}%)")

# ── Diagnosis ──────────────────────────────────────────────────────────
print()
print("=" * 90)
print("=== REMAINING ISSUES & RECOMMENDATIONS ===")
print("=" * 90)
print()

# Check if filter removal caused noise
raw_p2p_stat = []
cols9 = load("turret_diag_20260328_185638.csv")
wins9 = find_windows(cols9)
for s, e in wins9:
    settle = min(50, (e - s) // 4)
    sl = slice(s + settle, e)
    if len(cols9["timestamp"][sl]) < 20: continue
    raw_p2p_stat.append(np.ptp(cols9["raw_turret_angle"][sl]))
if raw_p2p_stat:
    avg_raw = np.mean(raw_p2p_stat)
    print(f"  Raw angle noise (stationary): {avg_raw:.4f} deg p2p avg")
    if avg_raw > 0.5:
        print(f"    WARNING: Pose noise visible without filter. Consider alpha=0.90.")
    else:
        print(f"    Minimal — removing the filter is safe.")
    print()

if v9["stat_sign"] > 20:
    print(f"  STATIONARY: sign_chg={v9['stat_sign']:.1f}% — kS=1.2 may be overshooting.")
    print(f"    Error oscillates around zero. Consider backing off to kS=1.1.")
elif v9["stat_sign"] < 5 and v9["stat_err"] > 0.3:
    print(f"  STATIONARY: |err|={v9['stat_err']:.3f}d, still one-sided (sign_chg={v9['stat_sign']:.1f}%).")
    print(f"    kS=1.2 still not enough, or kI=0.5 needs more time.")
    print(f"    -> Try kI=0.8 to accumulate faster.")
elif v9["stat_err"] < 0.3:
    print(f"  STATIONARY: |err|={v9['stat_err']:.3f}d — excellent precision!")
else:
    print(f"  STATIONARY: |err|={v9['stat_err']:.3f}d, sign_chg={v9['stat_sign']:.1f}%")
    print(f"    Getting close to optimal. Minor tuning may help.")

print()
if v9["move_err"] > 5:
    print(f"  MOTION: |err|={v9['move_err']:.2f}d — dominated by extreme spin episodes.")
    print(f"    At the hardware speed limit. No further software fix available.")
    print(f"    For moderate speeds (<1 rad/s), check omega table above.")
else:
    print(f"  MOTION: |err|={v9['move_err']:.2f}d — good tracking!")

print()
if v9["windup_post"] > 20:
    print(f"  WINDUP: {v9['windup_post']:.1f}d after dir changes — still present during extreme spins.")
else:
    print(f"  WINDUP: {v9['windup_post']:.1f}d — well controlled.")
print()
