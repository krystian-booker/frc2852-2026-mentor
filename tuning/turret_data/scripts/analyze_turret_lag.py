"""
Turret Lag Analysis — Moving vs Stationary
Investigates whether turret position lags behind setpoint during robot motion.
"""
import csv
import numpy as np

CSV_FILE = "turret_diag_20260328_170907.csv"

with open(CSV_FILE, "r") as f:
    reader = csv.DictReader(f)
    rows = list(reader)

cols = {key: np.array([float(row[key]) for row in rows]) for key in rows[0]}
n = len(rows)

# Skip first 2s settling
t0 = cols["timestamp"][0]
settled = cols["timestamp"] > (t0 + 2.0)

speed = np.sqrt(cols["vel_vx"]**2 + cols["vel_vy"]**2)
omega = np.abs(cols["vel_omega"])

# Classify: stationary vs moving (translating or rotating)
stationary = settled & (speed < 0.05) & (omega < 0.05)
moving = settled & ((speed >= 0.1) | (omega >= 0.1))

error = cols["turret_error"]
abs_error = np.abs(error)
setpoint = cols["turret_setpoint"]
actual = cols["turret_actual"]

print("=== Turret Lag Analysis: Moving vs Stationary ===\n")

print(f"Total samples after settle: {np.sum(settled)}")
print(f"Stationary samples: {np.sum(stationary)}")
print(f"Moving samples: {np.sum(moving)}")
print()

# ── Compare error magnitude ────────────────────────────────────────────
print("=== ERROR COMPARISON ===")
print(f"{'':30s} {'Stationary':>12s} {'Moving':>12s}")
print(f"  {'mean |error|':30s} {np.mean(abs_error[stationary]):12.4f} {np.mean(abs_error[moving]):12.4f} deg")
print(f"  {'std |error|':30s} {np.std(abs_error[stationary]):12.4f} {np.std(abs_error[moving]):12.4f} deg")
print(f"  {'max |error|':30s} {np.max(abs_error[stationary]):12.4f} {np.max(abs_error[moving]):12.4f} deg")
print(f"  {'median |error|':30s} {np.median(abs_error[stationary]):12.4f} {np.median(abs_error[moving]):12.4f} deg")
print(f"  {'p95 |error|':30s} {np.percentile(abs_error[stationary], 95):12.4f} {np.percentile(abs_error[moving], 95):12.4f} deg")
print()

# ── Does error correlate with speed? ───────────────────────────────────
print("=== ERROR vs SPEED CORRELATION ===")
m = settled & (speed > 0.01)  # Exclude dead-zero speed samples
if np.sum(m) > 20:
    r = np.corrcoef(speed[m], abs_error[m])[0, 1]
    print(f"  |error| vs speed correlation: r = {r:+.4f}")

r_omega = np.corrcoef(omega[settled], abs_error[settled])[0, 1]
print(f"  |error| vs |omega| correlation: r = {r_omega:+.4f}")
print()

# ── Error by speed bucket ──────────────────────────────────────────────
print("=== ERROR BY SPEED BUCKET ===")
print(f"  {'Speed range':20s} {'Samples':>8s} {'Mean |err|':>12s} {'P95 |err|':>12s} {'Mean setpoint chg':>18s}")
buckets = [(0, 0.05), (0.05, 0.2), (0.2, 0.5), (0.5, 1.0), (1.0, 2.0), (2.0, 99)]
for lo, hi in buckets:
    mask = settled & (speed >= lo) & (speed < hi)
    cnt = np.sum(mask)
    if cnt < 5:
        continue
    # Setpoint rate of change (deg/s)
    sp_diff = np.abs(np.diff(setpoint))
    dt = np.diff(cols["timestamp"])
    sp_rate = np.zeros(n)
    sp_rate[1:] = sp_diff / np.maximum(dt, 0.001)
    print(f"  {f'{lo:.2f}-{hi:.2f} m/s':20s} {cnt:8d} {np.mean(abs_error[mask]):12.4f} {np.percentile(abs_error[mask], 95):12.4f} {np.mean(sp_rate[mask]):18.2f} deg/s")
print()

# ── Setpoint rate of change analysis ───────────────────────────────────
print("=== SETPOINT RATE OF CHANGE ===")
sp_diff = np.diff(setpoint)
dt = np.diff(cols["timestamp"])
sp_rate = sp_diff / np.maximum(dt, 0.001)  # deg/s

# Pad to match original array length
sp_rate_full = np.zeros(n)
sp_rate_full[1:] = sp_rate

print(f"  {'':30s} {'Stationary':>12s} {'Moving':>12s}")
print(f"  {'mean |setpoint rate|':30s} {np.mean(np.abs(sp_rate_full[stationary])):12.2f} {np.mean(np.abs(sp_rate_full[moving])):12.2f} deg/s")
print(f"  {'max |setpoint rate|':30s} {np.max(np.abs(sp_rate_full[stationary])):12.2f} {np.max(np.abs(sp_rate_full[moving])):12.2f} deg/s")
print(f"  {'p95 |setpoint rate|':30s} {np.percentile(np.abs(sp_rate_full[stationary]), 95):12.2f} {np.percentile(np.abs(sp_rate_full[moving]), 95):12.2f} deg/s")
print()

# Does error correlate with setpoint rate of change?
m2 = settled & (np.abs(sp_rate_full) > 0.1)
if np.sum(m2) > 20:
    r_rate = np.corrcoef(np.abs(sp_rate_full[m2]), abs_error[m2])[0, 1]
    print(f"  |error| vs |setpoint_rate| correlation: r = {r_rate:+.4f}")
print()

# ── SOTM analysis during motion ────────────────────────────────────────
print("=== SOTM DURING MOTION ===")
sotm_moving = cols["sotm_active"][moving]
print(f"  SOTM active during motion: {np.mean(sotm_moving)*100:.1f}% of moving samples")
sotm_on = moving & (cols["sotm_active"] > 0.5)
sotm_off = moving & (cols["sotm_active"] < 0.5)
if np.sum(sotm_on) > 5 and np.sum(sotm_off) > 5:
    print(f"  Mean |error| with SOTM on:  {np.mean(abs_error[sotm_on]):.4f} deg")
    print(f"  Mean |error| with SOTM off: {np.mean(abs_error[sotm_off]):.4f} deg")
print()

# ── Error sign analysis during motion ──────────────────────────────────
print("=== ERROR DIRECTION DURING MOTION ===")

# When the setpoint is changing (moving), is the error consistently in
# the direction of "lagging behind"?
# Lag means: turret hasn't caught up yet, so error = setpoint - actual
# has the same sign as the direction of setpoint change.

lag_count = 0
lead_count = 0
neutral_count = 0
m_moving_idx = np.where(moving)[0]
for i in m_moving_idx:
    if i == 0:
        continue
    sp_change = setpoint[i] - setpoint[i-1]
    if abs(sp_change) < 0.001:
        neutral_count += 1
        continue
    err = error[i]
    # Lag: error same sign as setpoint change direction
    # (setpoint moved positive, actual hasn't caught up, error is positive)
    if np.sign(sp_change) == np.sign(err):
        lag_count += 1
    else:
        lead_count += 1

total = lag_count + lead_count
if total > 0:
    print(f"  Turret LAGGING behind setpoint: {lag_count}/{total} ({100*lag_count/total:.1f}%)")
    print(f"  Turret LEADING setpoint:        {lead_count}/{total} ({100*lead_count/total:.1f}%)")
    print(f"  No setpoint change:             {neutral_count}")
    if lag_count > total * 0.6:
        print(f"  -> CONFIRMED: Turret consistently lags behind moving setpoint")
    elif lead_count > total * 0.6:
        print(f"  -> Turret tends to overshoot (lead)")
    else:
        print(f"  -> No strong directional bias")
print()

# ── Transition analysis: moving -> stationary ──────────────────────────
print("=== TRANSITION: MOVING -> STATIONARY (settling) ===")

# Find transitions from moving to stationary
transitions = []
for i in range(1, n):
    if moving[i-1] and stationary[i]:
        transitions.append(i)

print(f"  Found {len(transitions)} moving->stationary transitions")

if transitions:
    settle_times = []
    for t_idx in transitions:
        # Look at error from transition point forward, find when |error| < 0.5 deg
        # for 10 consecutive samples
        settled_at = None
        for j in range(t_idx, min(t_idx + 200, n)):
            if j + 10 >= n:
                break
            window_err = abs_error[j:j+10]
            if np.all(window_err < 1.0):
                settled_at = j
                break
        if settled_at is not None:
            dt_settle = cols["timestamp"][settled_at] - cols["timestamp"][t_idx]
            settle_times.append(dt_settle)
            err_at_transition = abs_error[t_idx]
            err_after_settle = np.mean(abs_error[settled_at:settled_at+10])

    if settle_times:
        print(f"  Avg settling time (to <1 deg): {np.mean(settle_times):.3f}s")
        print(f"  Max settling time:             {np.max(settle_times):.3f}s")
        print(f"  Min settling time:             {np.min(settle_times):.3f}s")
print()

# ── MotionMagic bandwidth check ────────────────────────────────────────
print("=== MOTION MAGIC BANDWIDTH CHECK ===")
print("  Configured: cruise=5.0 rot/s (1800 deg/s), accel=25 rot/s^2 (9000 deg/s^2)")
print(f"  Max observed setpoint rate: {np.max(np.abs(sp_rate_full[moving])):.1f} deg/s")
print(f"  P95 observed setpoint rate: {np.percentile(np.abs(sp_rate_full[moving]), 95):.1f} deg/s")
if np.max(np.abs(sp_rate_full[moving])) > 1800:
    print("  WARNING: Setpoint rate exceeds MotionMagic cruise velocity!")
elif np.percentile(np.abs(sp_rate_full[moving]), 95) > 500:
    print("  Note: High setpoint rates — MotionMagic profile may be rate-limiting turret tracking")
else:
    print("  Setpoint rates are well within MotionMagic limits")
print()

# ── Filter lag analysis ────────────────────────────────────────────────
print("=== LOW-PASS FILTER LAG ANALYSIS ===")
raw = cols["raw_turret_angle"]
filt = cols["filtered_turret_angle"]

raw_rate_full = np.zeros(n)
raw_rate_full[1:] = np.diff(raw) / np.maximum(np.diff(cols["timestamp"]), 0.001)

filt_rate_full = np.zeros(n)
filt_rate_full[1:] = np.diff(filt) / np.maximum(np.diff(cols["timestamp"]), 0.001)

# Filter lag = raw - filtered (positive means filter lags behind)
filter_lag = raw - filt
print(f"  {'':30s} {'Stationary':>12s} {'Moving':>12s}")
print(f"  {'mean (raw - filtered)':30s} {np.mean(filter_lag[stationary]):12.4f} {np.mean(filter_lag[moving]):12.4f} deg")
print(f"  {'std (raw - filtered)':30s} {np.std(filter_lag[stationary]):12.4f} {np.std(filter_lag[moving]):12.4f} deg")
print(f"  {'max |raw - filtered|':30s} {np.max(np.abs(filter_lag[stationary])):12.4f} {np.max(np.abs(filter_lag[moving])):12.4f} deg")
print(f"  {'p95 |raw - filtered|':30s} {np.percentile(np.abs(filter_lag[stationary]), 95):12.4f} {np.percentile(np.abs(filter_lag[moving]), 95):12.4f} deg")
print()

# Does filter lag correlate with setpoint rate?
m3 = moving & (np.abs(sp_rate_full) > 1.0)
if np.sum(m3) > 20:
    r_filt_lag = np.corrcoef(np.abs(filter_lag[m3]), np.abs(sp_rate_full[m3]))[0, 1]
    print(f"  |filter_lag| vs |setpoint_rate| correlation: r = {r_filt_lag:+.4f}")

# Is the filter lag in the same direction as motion?
lag_same_dir = 0
lag_opp_dir = 0
for i in np.where(m3)[0]:
    if np.sign(raw_rate_full[i]) == np.sign(filter_lag[i]) and abs(filter_lag[i]) > 0.01:
        lag_same_dir += 1
    elif abs(filter_lag[i]) > 0.01:
        lag_opp_dir += 1
total_filt = lag_same_dir + lag_opp_dir
if total_filt > 0:
    print(f"  Filter lags in direction of motion: {lag_same_dir}/{total_filt} ({100*lag_same_dir/total_filt:.1f}%)")
print()

# ── Diagnosis ──────────────────────────────────────────────────────────
print("=" * 70)
print("=== DIAGNOSIS ===")
print("=" * 70)

mean_err_stat = np.mean(abs_error[stationary])
mean_err_move = np.mean(abs_error[moving])
err_ratio = mean_err_move / max(mean_err_stat, 0.001)

print(f"\n  Error while stationary: {mean_err_stat:.3f} deg avg")
print(f"  Error while moving:     {mean_err_move:.3f} deg avg")
print(f"  Ratio (moving/stationary): {err_ratio:.1f}x")
print()

max_filter_lag_moving = np.max(np.abs(filter_lag[moving]))
mean_filter_lag_moving = np.mean(np.abs(filter_lag[moving]))
p95_sp_rate = np.percentile(np.abs(sp_rate_full[moving]), 95)

issues = []

if mean_filter_lag_moving > 0.5:
    issues.append(("LOW-PASS FILTER LAG", mean_filter_lag_moving,
        f"The SMOOTHING_ALPHA=0.15 low-pass filter adds {mean_filter_lag_moving:.2f} deg avg lag during motion\n"
        f"    (max {max_filter_lag_moving:.2f} deg). The filter smooths the setpoint but also delays it.\n"
        f"    At {p95_sp_rate:.0f} deg/s setpoint rate, alpha=0.15 means the filter output\n"
        f"    lags ~{0.85/50 * p95_sp_rate:.1f} deg behind the true angle (0.85/50Hz * rate).\n"
        f"    -> This is a SEPARATE issue from kD. Adding kD won't fix filter lag.\n"
        f"    -> Fix: raise SMOOTHING_ALPHA toward 0.3-0.5, or replace with a predictive filter."))

if err_ratio > 2.0 and mean_err_move > 1.5:
    issues.append(("MOTION TRACKING ERROR", mean_err_move,
        f"Error is {err_ratio:.1f}x worse during motion ({mean_err_move:.2f} vs {mean_err_stat:.2f} deg).\n"
        f"    The turret consistently lags behind the moving setpoint."))

if total > 0 and lag_count > total * 0.6:
    issues.append(("DIRECTIONAL LAG CONFIRMED", 100*lag_count/total,
        f"Turret lags in the direction of setpoint change {100*lag_count/total:.0f}% of the time.\n"
        f"    This is classic underdamped tracking behavior."))

if not issues:
    print("  No significant motion lag detected.")
else:
    for name, sev, desc in issues:
        print(f"  [{name}]")
        print(f"    {desc}")
        print()

print("=" * 70)
print("=== SUMMARY ===")
print("=" * 70)
print()
if mean_filter_lag_moving > 0.5:
    print("  The turret lag during motion has TWO contributing factors:")
    print()
    print("  1. LOW-PASS FILTER (SMOOTHING_ALPHA=0.15) — software lag")
    print("     The filter delays the setpoint by smoothing it. During fast motion,")
    print("     the filtered angle trails the true calculated angle. This means the")
    print("     motor is being told to go to the WRONG position (a position the turret")
    print("     should have been at ~17ms ago). Adding kD will NOT fix this.")
    print("     -> Fix: increase SMOOTHING_ALPHA (less filtering, less lag)")
    print("        or add a lead-compensator/predictive element to the filter.")
    print()
    print("  2. MOTOR PID (kD=0) — mechanical lag")
    print("     Even if the setpoint were perfect, kP-only control overshoots and")
    print("     oscillates around changing setpoints. kD damps this.")
    print("     -> Fix: add kD=1.0 (as already planned)")
else:
    print("  Filter lag is minimal. The motion tracking error is likely dominated")
    print("  by the PID response (kD=0). Adding kD should help both the stationary")
    print("  jitter AND the motion tracking lag.")
print()
