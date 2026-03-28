"""
v7 analysis: kS=0.85, kI=0.5, kD=1.5
Compare v1 (original), v6 (last), v7 (current)
"""
import csv
import numpy as np

FILES = {
    "v1_original": "turret_diag_20260328_170907.csv",
    "v6_last":     "turret_diag_20260328_182848.csv",
    "v7_current":  "turret_diag_20260328_183800.csv",
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
        if len(cols["timestamp"][sl]) < 20: continue
        w_err = err[sl]
        w_abs = np.abs(w_err)
        sc = np.sum(np.diff(np.sign(w_err)) != 0) / max(len(w_err) - 1, 1) * 100
        stat_errs.extend(w_abs.tolist())
        win_details.append({
            "dur": cols["timestamp"][e-1] - cols["timestamp"][s],
            "err_mean": np.mean(w_abs), "bias": np.mean(w_err),
            "act_p2p": np.ptp(act[sl]), "sign_chg": sc,
        })
    stat_errs = np.array(stat_errs) if stat_errs else np.array([0])

    omega_err = {}
    for lo, hi in [(0, 0.1), (0.1, 0.5), (0.5, 1.0), (1.0, 2.0), (2.0, 99)]:
        mask = settled & (omega >= lo) & (omega < hi)
        if np.sum(mask) < 5: continue
        omega_err[f"{lo}-{hi}"] = {
            "cnt": int(np.sum(mask)),
            "err": np.mean(abs_err[mask]),
            "p95": np.percentile(abs_err[mask], 95),
            "act_rate": np.mean(np.abs(act_rate[mask])),
        }

    sp_dir = np.zeros(n, dtype=bool)
    for i in range(2, n):
        if sp_rate[i] * sp_rate[i-1] < 0 and abs(sp_rate[i]) > 5: sp_dir[i] = True
    pre, post = [], []
    for i in np.where(sp_dir)[0]:
        if 10 < i < n - 10:
            pre.append(np.mean(abs_err[max(0, i-10):i]))
            post.append(np.mean(abs_err[i:min(n, i+10)]))

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
        "omega": omega_err, "wins": win_details,
    }

R = {k: analyze(load(p)) for k, p in FILES.items()}
v1, v6, v7 = R["v1_original"], R["v6_last"], R["v7_current"]

# ── Main comparison ────────────────────────────────────────────────────
print("=" * 90)
print("=== v1 (original) vs v6 (kD=2.0) vs v7 (kS=0.85, kD=1.5) ===")
print("=" * 90)
print()
print(f"  {'Metric':35s} {'v1 original':>13s} {'v6 last':>13s} {'v7 current':>13s} {'v1->v7':>8s}")
print(f"  {'-'*35} {'-'*13} {'-'*13} {'-'*13} {'-'*8}")

def row(name, k, u="deg"):
    a, b, c = v1[k], v6[k], v7[k]
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
print()
print("  INTEGRAL WINDUP:")
row("  err before dir change", "windup_pre")
row("  err after dir change", "windup_post")

# ── v6 vs v7 omega ────────────────────────────────────────────────────
print()
print("=" * 90)
print("=== ERROR BY OMEGA: v6 (kD=2.0) vs v7 (kD=1.5) ===")
print("=" * 90)
print(f"  {'Omega':12s} {'v6 |Err|':>9s} {'v7 |Err|':>9s} {'Chg':>7s}  "
      f"{'v6 ActRate':>10s} {'v7 ActRate':>10s} {'Chg':>7s}")
for k in sorted(set(list(v6["omega"].keys()) + list(v7["omega"].keys()))):
    o6, o7 = v6["omega"].get(k), v7["omega"].get(k)
    if not o6 or not o7: continue
    ep = (o7["err"] - o6["err"]) / o6["err"] * 100 if o6["err"] > 0 else 0
    rp = (o7["act_rate"] - o6["act_rate"]) / o6["act_rate"] * 100 if o6["act_rate"] > 0 else 0
    print(f"  {k:12s} {o6['err']:8.2f}d {o7['err']:8.2f}d {ep:+6.0f}%  "
          f"{o6['act_rate']:9.1f}d {o7['act_rate']:9.1f}d {rp:+6.0f}%")

# ── v7 per-window ─────────────────────────────────────────────────────
print()
print("=" * 90)
print("=== v7 PER-WINDOW STATIONARY DETAIL ===")
print("=" * 90)
print(f"  {'Win':4s} {'Dur':>5s} {'|Err|':>9s} {'Bias':>9s} {'Act p2p':>9s} {'Sign%':>7s}")
for i, w in enumerate(v7["wins"]):
    print(f"  {i:4d} {w['dur']:4.1f}s {w['err_mean']:8.4f}d {w['bias']:+8.4f}d "
          f"{w['act_p2p']:8.4f}d {w['sign_chg']:6.1f}%")

# ── Diagnosis ──────────────────────────────────────────────────────────
print()
print("=" * 90)
print("=== DIAGNOSIS ===")
print("=" * 90)
print()
print("Current: kP=80, kI=0.5, kD=1.5, kS=0.85, alpha=0.80, SyncCANcoder, PositionVoltage")
print()

# Summarize what's good vs what needs work
stat_ok = v7["stat_err"] < 0.5
motion_ok = v7["move_err"] < 5
windup_ok = v7["windup_post"] < v7["windup_pre"] * 2 or v7["windup_post"] < 10

# Stationary
if v7["stat_sign"] > 15:
    print(f"  STATIONARY: |err|={v7['stat_err']:.3f}d, sign_chg={v7['stat_sign']:.1f}%")
    print(f"    Error is crossing zero — kI+kS are overcoming friction.")
    if v7["stat_err"] > 0.5:
        print(f"    But still oscillating too much. kI or kP may be slightly high.")
        print(f"    -> Try reducing kI to 0.3 or kP to 60")
    else:
        print(f"    Precision looks good!")
elif v7["stat_sign"] < 5:
    print(f"  STATIONARY: |err|={v7['stat_err']:.3f}d, bias={v7['stat_bias']:+.3f}d, sign_chg={v7['stat_sign']:.1f}%")
    print(f"    Error still one-sided — need more friction compensation.")
    print(f"    -> Try kS=1.0 or kI=0.8")
else:
    print(f"  STATIONARY: |err|={v7['stat_err']:.3f}d, sign_chg={v7['stat_sign']:.1f}%")
    print(f"    Starting to cross zero sometimes — getting close to balanced.")

print()

# Windup
if v7["windup_post"] > 20:
    ratio = v7["windup_post"] / max(v7["windup_pre"], 0.01)
    print(f"  WINDUP: {v7['windup_pre']:.1f}d -> {v7['windup_post']:.1f}d ({ratio:.1f}x)")
    if ratio > 2:
        print(f"    Still significant. Consider software iZone: only accumulate kI when |err|<5d")
    else:
        print(f"    Ratio is manageable. Dominated by physical speed limits during extreme spins.")
else:
    print(f"  WINDUP: well controlled ({v7['windup_post']:.1f}d after dir changes)")

print()

# Motion
if v7["move_err"] > 5:
    print(f"  MOTION: |err|={v7['move_err']:.2f}d (dominated by extreme spin episodes)")
    print(f"    At moderate speeds (<1 rad/s omega), check the omega table above.")
    print(f"    Extreme spin errors are largely a physical speed limit.")
else:
    print(f"  MOTION: |err|={v7['move_err']:.2f}d - good tracking")

print()

# Summary scorecard
print("=" * 90)
print("=== PROGRESS SCORECARD (v1 -> v7) ===")
print("=" * 90)
print()
metrics = [
    ("Stationary |error|", v1["stat_err"], v7["stat_err"], "deg", True),
    ("Stationary actual p2p", v1["stat_act_p2p"], v7["stat_act_p2p"], "deg", True),
    ("Filter lag (motion)", v1["filt_lag"], v7["filt_lag"], "deg", True),
    ("Moving |error| mean", v1["move_err"], v7["move_err"], "deg", True),
    ("Windup (post dir change)", v1["windup_post"], v7["windup_post"], "deg", True),
]
for name, before, after, unit, lower_better in metrics:
    pct = (after - before) / before * 100 if before != 0 else 0
    good = (after < before * 0.9) if lower_better else (after > before * 1.1)
    bad = (after > before * 1.1) if lower_better else (after < before * 0.9)
    icon = "OK" if good else ("!!" if bad else "~~")
    print(f"  [{icon}] {name:35s} {before:8.3f} -> {after:8.3f} {unit}  ({pct:+.0f}%)")
print()
