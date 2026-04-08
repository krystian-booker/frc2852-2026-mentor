# SOTM Tuning Guide

All constants are in `Constants.java` > `TurretAimingConstants`. The turret feedforward sign is in `RobotContainer.java`.

## Prerequisites

- Static shots (robot stationary) must already be accurate. SOTM builds on top of your calibrated lookup tables — if static shots are off, fix those first.
- Ensure odometry/pose estimation is reliable (QuestNav seeded, vision working).

## Step 1: Verify No Regression

Set `SOTM_ENABLED = false`, deploy, and shoot from several stationary positions. Confirm shots are identical to before the SOTM code was added. If anything changed, something is wired wrong — debug before proceeding.

## Step 2: Enable SOTM, Test Stationary

Set `SOTM_ENABLED = true` but keep the robot still. The virtual target offset should be ~0 (velocity is zero), so shots should be identical to static. If accuracy degrades, check that `speedsSupplier` isn't returning stale or noisy data.

## Step 3: Tune Ball Speed (Translation)

This is the most important constant: `AVERAGE_BALL_SPEED_MPS` (default 12.0).

1. Place a target at a known distance (~3-5 meters).
2. Drive toward the target at a steady ~1 m/s and shoot.
3. Observe where the ball lands:
   - **Ball goes long / overshoots** -> The ToF estimate is too high, meaning ball speed is set too low. **Increase** `AVERAGE_BALL_SPEED_MPS`.
   - **Ball falls short / undershoots** -> The ToF estimate is too low, meaning ball speed is set too high. **Decrease** `AVERAGE_BALL_SPEED_MPS`.
4. Repeat while driving perpendicular to the target (strafing). The same tuning logic applies — left/right miss means the same thing as over/undershoot.
5. Test at different distances (close and far) and speeds. A single constant won't be perfect everywhere, so aim for the best compromise across your typical shooting range.

### Quick reference

| Symptom while driving | Fix |
|---|---|
| Shots consistently lead the target (land ahead of where you're going) | Increase `AVERAGE_BALL_SPEED_MPS` |
| Shots consistently trail the target (land behind) | Decrease `AVERAGE_BALL_SPEED_MPS` |
| Accurate at close range but off at long range | The linear ToF model is a simplification — consider making `estimateTimeOfFlight()` nonlinear later |

## Step 4: Tune Turret Feedforward (Rotation)

This compensates for chassis yaw so the turret holds its field heading while the robot spins.

1. Aim at a static target from ~3 meters.
2. Slowly spin the robot in place (~0.5 rad/s). Watch the turret — it should stay locked on the target.
3. Observe:
   - **Turret holds steady on target** -> Sign and magnitude are correct. Done.
   - **Turret drifts in the same direction as chassis rotation** (doubles the error) -> The sign is wrong. In `RobotContainer.java`, flip `-chassisOmega` to `+chassisOmega`.
   - **Turret mostly tracks but lags behind** -> The PID is doing most of the work. This is acceptable — the feedforward just helps it settle faster.
   - **Turret oscillates or overshoots while spinning** -> The feedforward is too aggressive. Scale it down, e.g., `turret.setPosition(turretSetpoint, -chassisOmega * 0.7)`.

## Step 5: Tune Safety Clamp

`SOTM_MAX_LEAD_METERS` (default 1.5) limits how far the virtual target can shift from the real target. This prevents wild aiming at high speeds.

- At your robot's max speed (~5 m/s) with a typical ToF (~0.4s), the raw lead would be ~2m, which gets clamped to 1.5m.
- If shots at high speed are consistently off in the same direction, **increase** this value.
- If the turret aims at unreasonable positions at high speed, **decrease** it.
- A good range is 1.0 - 2.5 meters.

## Telemetry to Watch

These are already published or easily observable:
- `Shoot/TargetRPM` and `Shoot/TargetHoodAngle` — should change smoothly while driving
- Turret position vs target — should track closely even while spinning
- `shooterCalculator.getLastDistanceMeters()` — distance to the virtual target (will differ from raw distance while moving)

## Summary of Constants

| Constant | Default | What it does |
|---|---|---|
| `SOTM_ENABLED` | `true` | Master on/off switch |
| `AVERAGE_BALL_SPEED_MPS` | `12.0` | Controls ToF estimate and thus how far the virtual target shifts |
| `SOTM_MAX_LEAD_METERS` | `1.5` | Caps the virtual target offset for safety |
