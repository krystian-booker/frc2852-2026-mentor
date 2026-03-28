"""
Turret Jitter Diagnostic Analysis
Analyzes CSV data from DiagnosticLogger to identify the source of turret jitter.
"""
import csv
import numpy as np

CSV_FILE = "turret_diag_20260328_170907.csv"

# ── Load data ──────────────────────────────────────────────────────────────
with open(CSV_FILE, "r") as f:
    reader = csv.DictReader(f)
    rows = list(reader)

cols = {key: np.array([float(row[key]) for row in rows]) for key in rows[0]}
n = len(rows)
duration = cols["timestamp"][-1] - cols["timestamp"][0]

print(f"=== Turret Jitter Diagnostic Report ===")
print(f"File: {CSV_FILE}")
print(f"Rows: {n}  Duration: {duration:.1f}s  Avg rate: {n/duration:.1f} Hz")
print()

# ── Find contiguous stationary windows ──────────────────────────────────
speed = np.sqrt(cols["vel_vx"]**2 + cols["vel_vy"]**2)
stationary = (speed < 0.05) & (np.abs(cols["vel_omega"]) < 0.05)

# Find contiguous runs of stationary=True
windows = []
start = None
for i in range(n):
    if stationary[i] and start is None:
        start = i
    elif not stationary[i] and start is not None:
        windows.append((start, i))
        start = None
if start is not None:
    windows.append((start, n))

# Filter windows: at least 1 second of data (50 samples)
windows = [(s, e) for s, e in windows if (e - s) >= 50]
print(f"Found {len(windows)} stationary windows (>1s each):")
for i, (s, e) in enumerate(windows):
    t_start = cols["timestamp"][s] - cols["timestamp"][0]
    t_dur = cols["timestamp"][e-1] - cols["timestamp"][s]
    print(f"  Window {i}: rows {s}-{e-1}, t={t_start:.1f}s-{t_start+t_dur:.1f}s, "
          f"duration={t_dur:.1f}s, heading~{np.mean(cols['pose_heading'][s:e]):.1f} deg")
print()

# ── Analyze each stationary window ──────────────────────────────────────
def analyze_window(s, e, label):
    sl = slice(s, e)
    t = cols["timestamp"][sl]
    dur = t[-1] - t[0]
    n_samples = e - s

    # Skip first 1s of each window for settling
    settle_samples = min(50, n_samples // 4)
    sl = slice(s + settle_samples, e)
    t = cols["timestamp"][sl]
    if len(t) < 20:
        print(f"  {label}: Too few samples after settling, skipping.\n")
        return None

    dur = t[-1] - t[0]
    print(f"--- {label} ({e-s} samples, {dur:.1f}s after 1s settle trim) ---")

    def p(name, key, unit=""):
        d = cols[key][sl]
        u = f" {unit}" if unit else ""
        std = np.std(d)
        p2p = np.ptp(d)
        print(f"  {name:30s}  std={std:8.4f}{u}  p2p={p2p:8.4f}{u}  mean={np.mean(d):10.4f}{u}")
        return std, p2p

    print("  STAGE A - Pose:")
    heading_std, heading_p2p = p("pose_heading", "pose_heading", "deg")
    p("pose_x", "pose_x", "m")
    p("pose_y", "pose_y", "m")
    tag_std, _ = p("vision_tag_count", "vision_tag_count")
    p("vision_feeding", "vision_feeding")

    print("  STAGE B - Calculator:")
    raw_std, raw_p2p = p("raw_turret_angle", "raw_turret_angle", "deg")
    filt_std, filt_p2p = p("filtered_turret_angle", "filtered_turret_angle", "deg")
    p("distance", "distance", "m")
    p("sotm_active", "sotm_active")

    print("  STAGE C - Motor:")
    sp_std, sp_p2p = p("turret_setpoint", "turret_setpoint", "deg")
    act_std, act_p2p = p("turret_actual", "turret_actual", "deg")
    err_std, err_p2p = p("turret_error", "turret_error", "deg")
    volt_std, volt_p2p = p("motor_voltage", "motor_voltage", "V")
    cur_std, cur_p2p = p("stator_current", "stator_current", "A")

    # Error sign analysis: is the error consistently one direction or oscillating?
    err = cols["turret_error"][sl]
    mean_err = np.mean(err)
    sign_changes = np.sum(np.diff(np.sign(err)) != 0)

    print(f"\n  ERROR PATTERN:")
    print(f"    mean error = {mean_err:+.4f} deg (bias)")
    print(f"    error sign changes = {sign_changes} in {len(err)} samples "
          f"({sign_changes/len(err)*100:.1f}% of cycles)")
    if abs(mean_err) > 0.5 * err_std and err_std > 0.1:
        print(f"    -> Biased error: motor consistently lags behind setpoint")
    if sign_changes > len(err) * 0.3:
        print(f"    -> Error oscillates frequently: motor is hunting/overshooting")

    # Voltage pattern
    volt = cols["motor_voltage"][sl]
    volt_sign_changes = np.sum(np.diff(np.sign(volt)) != 0)
    print(f"\n  VOLTAGE PATTERN:")
    print(f"    mean voltage = {np.mean(volt):+.4f} V")
    print(f"    voltage sign changes = {volt_sign_changes} ({volt_sign_changes/len(volt)*100:.1f}% of cycles)")

    # Propagation analysis: compare p2p at each stage
    print(f"\n  JITTER PROPAGATION (peak-to-peak):")
    print(f"    pose_heading  -> raw_angle:     {heading_p2p:.4f} -> {raw_p2p:.4f} deg", end="")
    if heading_p2p > 0.01:
        print(f"  (amplification: {raw_p2p/heading_p2p:.2f}x)")
    else:
        print()
    print(f"    raw_angle     -> filtered_angle: {raw_p2p:.4f} -> {filt_p2p:.4f} deg", end="")
    if raw_p2p > 0.01:
        attn = (1 - filt_p2p / raw_p2p) * 100
        print(f"  (filter: {attn:+.0f}% attenuation)")
    else:
        print()
    print(f"    filtered/sp   -> turret_actual:  {sp_p2p:.4f} -> {act_p2p:.4f} deg", end="")
    if sp_p2p > 0.01:
        print(f"  (motor amplification: {act_p2p/sp_p2p:.2f}x)")
    else:
        print()

    print()
    return {
        "heading_p2p": heading_p2p, "heading_std": heading_std,
        "raw_p2p": raw_p2p, "raw_std": raw_std,
        "filt_p2p": filt_p2p, "filt_std": filt_std,
        "sp_p2p": sp_p2p, "sp_std": sp_std,
        "act_p2p": act_p2p, "act_std": act_std,
        "err_p2p": err_p2p, "err_std": err_std,
        "volt_std": volt_std, "mean_err": mean_err,
        "sign_changes_pct": sign_changes / len(err) * 100,
        "tag_std": tag_std,
    }

results = []
for i, (s, e) in enumerate(windows):
    r = analyze_window(s, e, f"Window {i}")
    if r:
        results.append((i, r))

if not results:
    print("No usable stationary windows found. Was the robot moving the entire time?")
    exit()

# ── Overall diagnosis across all windows ────────────────────────────────
print("=" * 70)
print("=== DIAGNOSIS (across all stationary windows) ===")
print("=" * 70)

# Average the metrics across windows
avg = {}
for key in results[0][1]:
    avg[key] = np.mean([r[key] for _, r in results])

print(f"\nAveraged metrics across {len(results)} windows:")
print(f"  pose_heading p2p:   {avg['heading_p2p']:.4f} deg  (std={avg['heading_std']:.4f})")
print(f"  raw_turret_angle:   {avg['raw_p2p']:.4f} deg  (std={avg['raw_std']:.4f})")
print(f"  filtered_angle:     {avg['filt_p2p']:.4f} deg  (std={avg['filt_std']:.4f})")
print(f"  turret_setpoint:    {avg['sp_p2p']:.4f} deg  (std={avg['sp_std']:.4f})")
print(f"  turret_actual:      {avg['act_p2p']:.4f} deg  (std={avg['act_std']:.4f})")
print(f"  turret_error:       {avg['err_p2p']:.4f} deg  (std={avg['err_std']:.4f})")

print()
issues = []

# Diagnosis 1: Pose heading noise
if avg["heading_p2p"] > 0.3:
    issues.append(("POSE HEADING NOISE", avg["heading_p2p"],
        f"Heading jitters by {avg['heading_p2p']:.3f} deg p2p while stationary.\n"
        f"    This feeds directly into the turret angle calculation.\n"
        f"    -> Root cause is likely in pose estimation (vision noise, single-tag ambiguity)."))

# Diagnosis 2: Filter effectiveness
if avg["raw_p2p"] > 0.1:
    if avg["filt_p2p"] > 0.5 * avg["raw_p2p"]:
        attn = (1 - avg["filt_p2p"] / avg["raw_p2p"]) * 100
        issues.append(("WEAK LOW-PASS FILTER", avg["filt_p2p"],
            f"Filter attenuates only {attn:.0f}% of raw angle noise.\n"
            f"    raw p2p={avg['raw_p2p']:.3f} -> filtered p2p={avg['filt_p2p']:.3f} deg.\n"
            f"    SMOOTHING_ALPHA=0.15 is too aggressive (passes too much noise).\n"
            f"    -> Lower SMOOTHING_ALPHA to 0.05-0.08 in TurretAimingCalculator."))

# Diagnosis 3: Motor hunting (setpoint stable, actual oscillates)
if avg["act_p2p"] > avg["sp_p2p"] * 1.5 and avg["sp_p2p"] < 1.0:
    issues.append(("MOTOR PID HUNTING", avg["act_p2p"],
        f"Setpoint stable (p2p={avg['sp_p2p']:.3f}) but actual oscillates (p2p={avg['act_p2p']:.3f}).\n"
        f"    kP=80 with kD=0 causes undamped oscillation around setpoint.\n"
        f"    -> Add kD (try 0.5-2.0) to damp overshoot."))

# Diagnosis 4: Motor hunting (actual amplifies setpoint jitter)
if avg["act_p2p"] > avg["sp_p2p"] * 1.3 and avg["sp_p2p"] > 0.5:
    issues.append(("MOTOR AMPLIFIES JITTER", avg["act_p2p"],
        f"Motor overshoots setpoint changes, amplifying jitter.\n"
        f"    Setpoint p2p={avg['sp_p2p']:.3f} -> actual p2p={avg['act_p2p']:.3f} deg.\n"
        f"    -> Add kD to damp response, or slow down MotionMagic acceleration."))

# Diagnosis 5: Setpoint jitter (calculator output is noisy)
if avg["sp_p2p"] > 1.0:
    issues.append(("SETPOINT JITTER FROM CALCULATOR", avg["sp_p2p"],
        f"Calculator commands vary by {avg['sp_p2p']:.3f} deg p2p while stationary.\n"
        f"    -> Fix upstream: reduce pose noise or strengthen filter."))

# Diagnosis 6: Steady-state error bias
if abs(avg["mean_err"]) > 0.5:
    issues.append(("STEADY-STATE ERROR BIAS", abs(avg["mean_err"]),
        f"Mean error = {avg['mean_err']:+.3f} deg. Motor consistently undershoots.\n"
        f"    With kI=0, there's no integral action to eliminate steady-state error.\n"
        f"    -> Add small kI (try 0.5-2.0) or increase kS."))

# Diagnosis 7: Vision tag count fluctuation
if avg["tag_std"] > 0.2:
    issues.append(("VISION TAG COUNT UNSTABLE", avg["tag_std"],
        f"Vision tag count fluctuates (std={avg['tag_std']:.3f}).\n"
        f"    Tags appearing/disappearing causes pose estimate jumps.\n"
        f"    -> Check camera exposure, mounting, and tag visibility."))

if not issues:
    print("No significant jitter issues detected.")
else:
    issues.sort(key=lambda x: x[1], reverse=True)
    print("Issues found (sorted by severity):\n")
    for i, (name, severity, desc) in enumerate(issues):
        marker = ">>>" if i == 0 else "   "
        print(f"  {marker} [{i+1}] {name}")
        print(f"      {desc}")
        print()

# ── Recommendation ──────────────────────────────────────────────────────
print("=" * 70)
print("=== RECOMMENDED FIXES (in priority order) ===")
print("=" * 70)
if issues:
    # Determine primary source
    has_pose = any("POSE" in name or "VISION" in name for name, _, _ in issues)
    has_filter = any("FILTER" in name for name, _, _ in issues)
    has_motor = any("MOTOR" in name or "PID" in name for name, _, _ in issues)
    has_error = any("STEADY" in name for name, _, _ in issues)

    fix_num = 1
    if has_pose:
        print(f"\n  {fix_num}. FIX POSE ESTIMATION")
        print(f"     - Ensure 2+ AprilTags are always visible (multi-tag mode)")
        print(f"     - Check camera exposure settings and mounting rigidity")
        print(f"     - Consider increasing vision std devs to trust odometry more")
        fix_num += 1
    if has_filter:
        print(f"\n  {fix_num}. STRENGTHEN LOW-PASS FILTER")
        print(f"     - In TurretAimingCalculator.java, change SMOOTHING_ALPHA from 0.15 to 0.05")
        print(f"     - This will slow turret response slightly but greatly reduce jitter")
        fix_num += 1
    if has_motor:
        print(f"\n  {fix_num}. ADD DERIVATIVE GAIN TO TURRET PID")
        print(f"     - In Constants.java TurretConstants, set kD = 1.0 (start conservative)")
        print(f"     - This damps oscillation around setpoint without affecting tracking")
        fix_num += 1
    if has_error:
        print(f"\n  {fix_num}. ADD POSITION DEADBAND")
        print(f"     - Stop commanding new positions when error < 0.5 deg")
        print(f"     - Or add small kI (0.5-1.0) to eliminate steady-state offset")
        fix_num += 1
print()
