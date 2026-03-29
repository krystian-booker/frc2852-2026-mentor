"""
Analyze v4 data with aggressive driving (fast spins).
Focus on: stationary precision, motion tracking by speed/omega bucket,
turret physical speed limits, and kI behavior.
"""
import csv
import numpy as np

CSV_FILE = "turret_diag_20260328_180101.csv"

with open(CSV_FILE) as f:
    rows = list(csv.DictReader(f))
cols = {k: np.array([float(r[k]) for r in rows]) for k in rows[0]}
n = len(rows)
dt = np.diff(cols["timestamp"])
duration = cols["timestamp"][-1] - cols["timestamp"][0]

speed = np.sqrt(cols["vel_vx"]**2 + cols["vel_vy"]**2)
omega = np.abs(cols["vel_omega"])
err = cols["turret_error"]
abs_err = np.abs(err)
sp = cols["turret_setpoint"]
act = cols["turret_actual"]

# Setpoint rate of change (deg/s)
sp_rate = np.zeros(n)
sp_rate[1:] = np.diff(sp) / np.maximum(dt, 0.001)

# Actual rate of change (deg/s)
act_rate = np.zeros(n)
act_rate[1:] = np.diff(act) / np.maximum(dt, 0.001)

t0 = cols["timestamp"][0]
settled = cols["timestamp"] > (t0 + 2.0)
stationary = settled & (speed < 0.05) & (omega < 0.05)
moving = settled & ((speed >= 0.1) | (omega >= 0.1))

print(f"=== v4 Analysis: Aggressive Driving Test ===")
print(f"File: {CSV_FILE}")
print(f"Rows: {n}  Duration: {duration:.1f}s  Rate: {n/duration:.1f} Hz")
print(f"Stationary: {np.sum(stationary)}  Moving: {np.sum(moving)}")
print()

# ── Stationary performance (kI=1.5 check) ──────────────────────────────
print("=" * 75)
print("=== STATIONARY PERFORMANCE (kI=1.5, kD=1.0, alpha=0.80) ===")
print("=" * 75)

# Find stationary windows
wins, start = [], None
for i in range(n):
    s = (speed[i] < 0.05) & (omega[i] < 0.05)
    if s and start is None: start = i
    elif not s and start is not None:
        wins.append((start, i)); start = None
if start is not None: wins.append((start, n))
wins = [(s, e) for s, e in wins if (e - s) >= 50]

print(f"\nFound {len(wins)} stationary windows (>1s)")
if wins:
    all_stat_err = []
    for i, (s, e) in enumerate(wins):
        settle = min(50, (e - s) // 4)
        sl = slice(s + settle, e)
        if len(cols["timestamp"][sl]) < 20: continue
        m_err = np.mean(np.abs(err[sl]))
        m_bias = np.mean(err[sl])
        m_act_p2p = np.ptp(act[sl])
        m_sp_p2p = np.ptp(sp[sl])
        sc = np.sum(np.diff(np.sign(err[sl])) != 0) / max(len(err[sl])-1, 1) * 100
        dur = cols["timestamp"][e-1] - cols["timestamp"][s]
        all_stat_err.extend(np.abs(err[sl]).tolist())
        print(f"  Win {i}: {dur:4.1f}s  |err|={m_err:.4f}°  bias={m_bias:+.4f}°  "
              f"act_p2p={m_act_p2p:.4f}°  sign_chg={sc:.1f}%")

    all_stat_err = np.array(all_stat_err)
    print(f"\n  Overall stationary: |err| mean={np.mean(all_stat_err):.4f}°  "
          f"p95={np.percentile(all_stat_err, 95):.4f}°  "
          f"max={np.max(all_stat_err):.4f}°")
print()

# ── Error by omega bucket (the key new analysis) ──────────────────────
print("=" * 75)
print("=== ERROR BY ROTATION SPEED (omega) ===")
print("=" * 75)
print(f"  {'Omega range':20s} {'Samples':>8s} {'|Err| mean':>11s} {'|Err| p95':>11s} "
      f"{'SP rate':>11s} {'Act rate':>11s} {'Rate deficit':>12s}")

omega_buckets = [(0, 0.1), (0.1, 0.5), (0.5, 1.0), (1.0, 2.0), (2.0, 4.0), (4.0, 99)]
for lo, hi in omega_buckets:
    mask = settled & (omega >= lo) & (omega < hi)
    cnt = np.sum(mask)
    if cnt < 5: continue
    mean_err = np.mean(abs_err[mask])
    p95_err = np.percentile(abs_err[mask], 95)
    mean_sp_rate = np.mean(np.abs(sp_rate[mask]))
    mean_act_rate = np.mean(np.abs(act_rate[mask]))
    deficit = mean_sp_rate - mean_act_rate
    label = f"{lo:.1f}-{hi:.1f} rad/s"
    print(f"  {label:20s} {cnt:8d} {mean_err:10.2f}° {p95_err:10.2f}° "
          f"{mean_sp_rate:10.1f}°/s {mean_act_rate:10.1f}°/s {deficit:+11.1f}°/s")
print()

# ── Error by translation speed ─────────────────────────────────────────
print("=" * 75)
print("=== ERROR BY TRANSLATION SPEED ===")
print("=" * 75)
print(f"  {'Speed range':20s} {'Samples':>8s} {'|Err| mean':>11s} {'|Err| p95':>11s}")

speed_buckets = [(0, 0.05), (0.05, 0.3), (0.3, 0.7), (0.7, 1.2), (1.2, 2.0), (2.0, 99)]
for lo, hi in speed_buckets:
    mask = settled & (speed >= lo) & (speed < hi)
    cnt = np.sum(mask)
    if cnt < 5: continue
    print(f"  {f'{lo:.2f}-{hi:.2f} m/s':20s} {cnt:8d} {np.mean(abs_err[mask]):10.2f}° "
          f"{np.percentile(abs_err[mask], 95):10.2f}°")
print()

# ── Turret physical speed limit analysis ───────────────────────────────
print("=" * 75)
print("=== TURRET SPEED LIMIT ANALYSIS ===")
print("=" * 75)

# MotionMagic cruise = 5.0 rot/s = 1800 deg/s at turret
cruise_deg_s = 5.0 * 360.0
print(f"  MotionMagic cruise velocity: {cruise_deg_s:.0f} deg/s")
print(f"  Max observed setpoint rate:  {np.max(np.abs(sp_rate[settled])):.1f} deg/s")
print(f"  P99 setpoint rate:           {np.percentile(np.abs(sp_rate[settled]), 99):.1f} deg/s")
print(f"  P95 setpoint rate:           {np.percentile(np.abs(sp_rate[settled]), 95):.1f} deg/s")
print(f"  Max observed actual rate:    {np.max(np.abs(act_rate[settled])):.1f} deg/s")
print(f"  P99 actual rate:             {np.percentile(np.abs(act_rate[settled]), 99):.1f} deg/s")
print()

# Find moments where setpoint rate exceeds actual rate significantly
# (turret can't keep up)
rate_deficit = np.abs(sp_rate) - np.abs(act_rate)
can_t_keep_up = settled & (np.abs(sp_rate) > 50) & (rate_deficit > 20)
print(f"  Samples where turret can't keep up (SP rate > actual rate + 20°/s): "
      f"{np.sum(can_t_keep_up)} ({100*np.sum(can_t_keep_up)/np.sum(settled):.1f}%)")
if np.sum(can_t_keep_up) > 0:
    print(f"    Mean |error| during these: {np.mean(abs_err[can_t_keep_up]):.2f}°")
    print(f"    Mean SP rate: {np.mean(np.abs(sp_rate[can_t_keep_up])):.1f} deg/s")
    print(f"    Mean actual rate: {np.mean(np.abs(act_rate[can_t_keep_up])):.1f} deg/s")
print()

# ── Setpoint rate vs robot omega relationship ──────────────────────────
print("=" * 75)
print("=== SETPOINT RATE vs ROBOT OMEGA ===")
print("=" * 75)
print("  (When robot spins, the turret must counter-rotate at the same rate)")
print()

# When robot rotates at omega rad/s, turret setpoint should change at
# omega * (180/pi) deg/s to counter-rotate
omega_deg = omega * 180 / np.pi  # convert to deg/s

m_rot = settled & (omega > 0.3)  # significant rotation
if np.sum(m_rot) > 10:
    r = np.corrcoef(omega_deg[m_rot], np.abs(sp_rate[m_rot]))[0, 1]
    print(f"  Robot omega vs |SP rate| correlation: r = {r:.4f}")
    print(f"  Mean robot rotation rate: {np.mean(omega_deg[m_rot]):.1f} deg/s")
    print(f"  Mean setpoint rate:       {np.mean(np.abs(sp_rate[m_rot])):.1f} deg/s")
    print(f"  Mean actual rate:         {np.mean(np.abs(act_rate[m_rot])):.1f} deg/s")
    print(f"  Mean |error|:             {np.mean(abs_err[m_rot]):.2f}°")

    # Check if setpoint rate matches omega (should be ~1:1 for counter-rotation)
    ratio = np.mean(np.abs(sp_rate[m_rot])) / np.mean(omega_deg[m_rot])
    print(f"  SP rate / omega ratio:    {ratio:.2f}x (should be ~1.0 for pure counter-rotation)")
print()

# ── Large error episodes ───────────────────────────────────────────────
print("=" * 75)
print("=== LARGE ERROR EPISODES (|error| > 10°) ===")
print("=" * 75)

big_err = settled & (abs_err > 10)
if np.sum(big_err) > 0:
    print(f"  Samples with |error| > 10°: {np.sum(big_err)} ({100*np.sum(big_err)/np.sum(settled):.1f}%)")
    print(f"  During these episodes:")
    print(f"    Mean omega:    {np.mean(omega[big_err]):.2f} rad/s ({np.mean(omega[big_err])*180/np.pi:.1f} deg/s)")
    print(f"    Mean speed:    {np.mean(speed[big_err]):.2f} m/s")
    print(f"    Mean |error|:  {np.mean(abs_err[big_err]):.1f}°")
    print(f"    Max |error|:   {np.max(abs_err[big_err]):.1f}°")
    print(f"    Mean SP rate:  {np.mean(np.abs(sp_rate[big_err])):.1f} deg/s")
    print(f"    Mean act rate: {np.mean(np.abs(act_rate[big_err])):.1f} deg/s")

    # How long do these episodes last?
    ep_starts = []
    in_ep = False
    for i in range(n):
        if big_err[i] and not in_ep:
            ep_starts.append(i)
            in_ep = True
        elif not big_err[i]:
            in_ep = False

    print(f"    Number of episodes: {len(ep_starts)}")
else:
    print("  No samples with |error| > 10°")
print()

# ── Filter lag during fast motion ──────────────────────────────────────
print("=" * 75)
print("=== FILTER LAG BY MOTION INTENSITY ===")
print("=" * 75)

filt_lag = cols["raw_turret_angle"] - cols["filtered_turret_angle"]
print(f"  {'Omega range':20s} {'Samples':>8s} {'|Filter lag|':>12s} {'|Error|':>10s}")
for lo, hi in omega_buckets:
    mask = settled & (omega >= lo) & (omega < hi)
    cnt = np.sum(mask)
    if cnt < 5: continue
    label = f"{lo:.1f}-{hi:.1f} rad/s"
    print(f"  {label:20s} {cnt:8d} {np.mean(np.abs(filt_lag[mask])):11.3f}° "
          f"{np.mean(abs_err[mask]):9.2f}°")
print()

# ── kI integral behavior check ─────────────────────────────────────────
print("=" * 75)
print("=== kI BEHAVIOR CHECK ===")
print("=" * 75)

# Look for integral windup signs: error stays same sign for a long time
# then overshoots when direction changes
# Check error during direction changes
sp_dir_change = np.zeros(n, dtype=bool)
for i in range(2, n):
    if sp_rate[i] * sp_rate[i-1] < 0 and abs(sp_rate[i]) > 5:
        sp_dir_change[i] = True

if np.sum(sp_dir_change) > 5:
    # Look at error in windows around direction changes
    pre_err = []
    post_err = []
    for i in np.where(sp_dir_change)[0]:
        if i > 10 and i < n - 10:
            pre_err.append(np.mean(abs_err[max(0,i-10):i]))
            post_err.append(np.mean(abs_err[i:min(n,i+10)]))
    if pre_err:
        print(f"  Setpoint direction changes: {np.sum(sp_dir_change)}")
        print(f"  Mean |error| before direction change: {np.mean(pre_err):.2f}°")
        print(f"  Mean |error| after direction change:  {np.mean(post_err):.2f}°")
        if np.mean(post_err) > np.mean(pre_err) * 1.5:
            print(f"  -> ERROR SPIKES after direction changes: possible integral windup")
            print(f"     Consider adding iZone or MaxIntegral to limit kI accumulation")
        else:
            print(f"  -> No integral windup detected")
else:
    print(f"  Not enough direction changes to analyze ({np.sum(sp_dir_change)})")
print()

# ── Diagnosis ──────────────────────────────────────────────────────────
print("=" * 75)
print("=== DIAGNOSIS & RECOMMENDATIONS ===")
print("=" * 75)
print()

stat_err = np.mean(all_stat_err) if wins else 0
move_err = np.mean(abs_err[moving]) if np.sum(moving) > 0 else 0
max_sp_rate = np.percentile(np.abs(sp_rate[settled]), 99)
max_act_rate = np.percentile(np.abs(act_rate[settled]), 99)

print(f"  Stationary |error|:  {stat_err:.3f}° (target: <0.5°)")
print(f"  Moving |error|:      {move_err:.2f}°")
print(f"  P99 setpoint rate:   {max_sp_rate:.0f} deg/s")
print(f"  P99 actual rate:     {max_act_rate:.0f} deg/s")
print(f"  MotionMagic cruise:  {cruise_deg_s:.0f} deg/s")
print()

if max_sp_rate > cruise_deg_s * 0.8:
    print(f"  [!] SETPOINT RATE NEAR CRUISE LIMIT")
    print(f"      P99 rate ({max_sp_rate:.0f}°/s) is {max_sp_rate/cruise_deg_s*100:.0f}% of cruise ({cruise_deg_s:.0f}°/s)")
    print(f"      The turret physically cannot spin fast enough for the fastest robot rotations.")
    print(f"      Options:")
    print(f"        a) Increase MotionMagic cruise velocity (if motor/gearing allows)")
    print(f"        b) Accept degraded tracking during extreme spins")
    print(f"        c) Limit robot rotation speed when shooting")
    print()

if max_act_rate < max_sp_rate * 0.7:
    print(f"  [!] TURRET SPEED SATURATION")
    print(f"      Actual rate ({max_act_rate:.0f}°/s) << setpoint rate ({max_sp_rate:.0f}°/s)")
    print(f"      During fast spins, the turret falls behind and builds up large errors.")
    print()
