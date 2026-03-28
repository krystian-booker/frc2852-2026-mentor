# Turret PID Tuning Guide

## Overview

This document describes the turret aiming diagnostic system and the tuning process used to optimize turret tracking precision. It covers the CSV diagnostic logger, the analysis scripts, the final tuned parameters, and how to re-validate after robot changes.

## Current Tuned Parameters

These are the final values in `Constants.java` `TurretConstants`:

| Parameter | Value | Purpose |
|-----------|-------|---------|
| kP | 80.0 | Proportional gain (V/rotation) |
| kI | 1.0 | Integral gain — overcomes position-dependent friction |
| kD | 1.5 | Derivative gain — damps overshoot and oscillation |
| kS | 1.2 | Static friction feedforward (V) — pre-compensates for stiction |
| kV | 4.8862 | Velocity feedforward |
| kA | 0.1 | Acceleration feedforward |
| kG | 0.0 | Gravity feedforward |

Other critical configuration (in `Turret.java` and `Constants.java`):

| Setting | Value | Why |
|---------|-------|-----|
| Feedback sensor | `SyncCANcoder` | Uses high-resolution internal rotor encoder for PID, CANcoder for absolute reference |
| Control mode | `PositionVoltage` | Direct PID — no MotionMagic rate limiting during tracking |
| Supply current limit | 60A | Allows full motor torque for fast slews |
| Stator current limit | 60A | Matches supply limit |
| Low-pass filter | Removed | Pose noise is small enough (~0.03 deg) that kD handles it |

## Diagnostic Logger

### How it works

`DiagnosticLogger.java` writes timestamped CSV files at 50Hz during teleop/auto. It's wired into the turret default command in `RobotContainer.java` and records 21 columns covering the full pipeline: pose estimation, aiming calculation, motor control, and vision state.

### Enabling/disabling

- **SmartDashboard toggle**: `Diagnostics/turret_diag` (boolean, defaults to enabled)
- **Compile-time**: `Constants.DiagnosticConstants.TURRET_LOGGING_DEFAULT_ENABLED`
- **Lifecycle**: Logging starts on `teleopInit()`/`autonomousInit()`, stops on `disabledInit()`
- **Files**: Written to `/home/lvuser/logs/turret_diag_YYYYMMDD_HHmmss.csv` on the roboRIO

### Retrieving logs

```bash
# From a laptop on the robot network:
scp lvuser@10.28.52.2:/home/lvuser/logs/turret_diag_*.csv .
# Or use FileZilla / WinSCP to the same path
```

### CSV columns

| # | Column | Unit | What it tells you |
|---|--------|------|-------------------|
| 0 | timestamp | seconds (FPGA) | Time reference |
| 1 | pose_x | meters | Pose stability |
| 2 | pose_y | meters | Pose stability |
| 3 | pose_heading | degrees | Heading jitter |
| 4 | vel_vx | m/s | Robot translational speed |
| 5 | vel_vy | m/s | Robot translational speed |
| 6 | vel_omega | rad/s | Robot rotational speed |
| 7 | target_x | meters | Target being aimed at |
| 8 | target_y | meters | Target switching detection |
| 9 | distance | meters | Distance to target |
| 10 | raw_turret_angle | degrees | Calculated turret angle |
| 11 | filtered_turret_angle | degrees | Same as raw (filter removed) |
| 12 | turret_setpoint | degrees | What motor is commanded |
| 13 | turret_actual | degrees | Motor encoder position |
| 14 | turret_error | degrees | Setpoint minus actual |
| 15 | motor_voltage | volts | Motor effort |
| 16 | stator_current | amps | Motor current draw |
| 17 | sotm_active | 0/1 | Shooting-on-the-move flag |
| 18 | vision_tag_count | count | Visible AprilTags |
| 19 | vision_feeding | 0/1 | Vision feeding pose estimator |
| 20 | cancoder_position | degrees | CANcoder raw position |

## Re-validation Procedure

Run this after any mechanical changes to the turret, drivetrain, or vision system.

### Step 1: Collect data

1. Deploy the robot code
2. Place robot where it can see AprilTags (2+ preferred)
3. Enable teleop
4. **Stationary test** (15 seconds): Let the robot sit still, auto-aiming at target
5. **Slow driving test** (15 seconds): Drive slowly while turret tracks
6. **Aggressive test** (15 seconds): Fast translation and robot spins
7. Disable — the CSV file is saved automatically

### Step 2: Pull the CSV

Copy the latest `turret_diag_*.csv` from `/home/lvuser/logs/` to the project root.

### Step 3: Run analysis

The analysis scripts are in the project root. Run against the new CSV by editing the filename in the script, or use the general-purpose ones:

```bash
# Quick stationary jitter check:
python analyze_turret_jitter.py

# Full comparison against a previous baseline:
python analyze_v9.py
```

Or ask Claude Code to analyze the new CSV — provide the filename and it will run the comparison against the original baseline data.

### Step 4: What to look for

**Healthy turret (current baseline performance):**

| Metric | Target | Acceptable |
|--------|--------|------------|
| Stationary actual p2p | 0.000 deg | < 0.2 deg |
| Stationary \|error\| mean | < 0.5 deg | < 1.5 deg |
| Stationary sign changes | > 5% | 0% means kI/kS not overcoming friction |
| Windup (post dir change) | < 10 deg | < 50 deg |
| Error at omega 0.1-0.5 | < 5 deg | < 10 deg |

**Red flags that need re-tuning:**

| Symptom | Likely cause | Fix |
|---------|-------------|-----|
| Actual p2p > 0.3 deg while stationary | Motor PID oscillating | Increase kD |
| \|error\| > 2 deg, 0% sign changes | Friction not compensated | Increase kS or kI |
| \|error\| > 2 deg, >30% sign changes | Overshooting / oscillating | Decrease kI or kS, increase kD |
| Large error after direction changes | Integral windup | Decrease kI |
| Turret visibly slow during spins | Current limiting | Check current limits, check for mechanical binding |
| Raw angle noise > 0.5 deg stationary | Pose estimation degraded | Check cameras, check vision |

## Tuning History (2026-03-28)

### What we started with (problems)
- 2-5 degree jitter while stationary and locked on target
- Turret lagging behind during robot motion
- No derivative gain (kD=0) causing undamped oscillation
- FusedCANcoder limiting position resolution to 0.1758 deg steps
- MotionMagic rate-limiting turret response during tracking
- Conservative 20A current limits capping motor torque

### Changes made (in order)

1. **Added CSV diagnostic logging** — `DiagnosticLogger.java` recording 21 columns at 50Hz across the full turret pipeline
2. **Added kD=1.0** — Damped the motor oscillation (actual p2p was 6-10x the setpoint p2p)
3. **Increased filter alpha 0.15 → 0.40** — Reduced software lag during motion (was 4.4 deg)
4. **Added kI=0.5** — Started closing steady-state offset from friction
5. **Switched FusedCANcoder → SyncCANcoder** — Motor PID now uses internal rotor encoder (0.0035 deg resolution vs 0.1758 deg). Biggest single improvement: actual p2p went to 0.000 deg.
6. **Increased filter alpha to 0.60, then 0.80** — Further reduced motion lag
7. **Switched MotionMagicVoltage → PositionVoltage** — Removed motion profile rate-limiting during tracking. Turret can now respond at full motor capability.
8. **Increased kS 0.48 → 0.70 → 0.85 → 1.0 → 1.2** — Iteratively increased static friction compensation
9. **Tuned kD from 1.0 → 2.0 → 1.5** — Found 2.0 was overdamped, 1.5 is the sweet spot
10. **Increased current limits 20A/40A → 60A/60A** — Removed artificial torque cap from safe-testing era
11. **Removed low-pass filter entirely** — Pose noise is 0.025 deg, kD handles it
12. **Increased kI to 1.0** — Faster integral accumulation for friction compensation

### Key diagnostic insights

- **Stage C (motor PID) was the original jitter source**, not pose estimation or the aiming calculator. Setpoint was stable at 0.045 deg p2p but the motor amplified it to 0.27 deg.
- **FusedCANcoder quantized position at 0.1758 deg** (2x CANcoder tick). SyncCANcoder uses the rotor encoder through 50:1 for ~0.0035 deg resolution. The motor was actually controlling well internally, but the coarse feedback limited PID precision.
- **MotionMagic was the motion tracking bottleneck**. During fast robot spins, the turret demanded 478 deg/s but MotionMagic's trajectory replanning every 20ms limited actual rate to 90 deg/s despite 1800 deg/s cruise capability.
- **kS (static friction) is position-dependent**. A single kS value can't perfectly compensate friction everywhere, which is why kI is needed as the adaptive component.
- **The turret hits mechanical speed limits at ~930 deg/s** (Kraken X44 at 6000RPM through 50:1 gearing). Extreme robot spins will always outpace the turret — this is a hardware constraint, not a software one.

## File Reference

| File | Purpose |
|------|---------|
| `src/main/java/frc/robot/util/DiagnosticLogger.java` | CSV logging utility |
| `src/main/java/frc/robot/util/TurretAimingCalculator.java` | Aiming calculations |
| `src/main/java/frc/robot/subsystems/Turret.java` | Motor control (SyncCANcoder + PositionVoltage) |
| `src/main/java/frc/robot/Constants.java` | All tunable parameters (TurretConstants) |
| `src/main/java/frc/robot/RobotContainer.java` | Diagnostic logging wiring (turret default command) |
| `src/main/java/frc/robot/Robot.java` | Logging lifecycle (start/stop on enable/disable) |
| `analyze_turret_jitter.py` | Stationary jitter analysis |
| `analyze_turret_lag.py` | Motion tracking lag analysis |
| `analyze_final.py` | Full before/after comparison |
