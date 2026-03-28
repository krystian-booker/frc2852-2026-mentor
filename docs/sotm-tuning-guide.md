# Shooting on the Move (SOTM) Tuning Guide

## Overview

SOTM compensates for robot motion during ball flight using the **virtual target method**. Instead of aiming at the real target, the system aims at a calculated point so that the ball's field-relative trajectory (exit velocity + robot velocity) hits the actual goal.

All tunable parameters live in `Constants.java` → `SOTMConstants`.

## Before You Start

1. **Measure the flywheel wheel diameter** and update `FLYWHEEL_DIAMETER_METERS` in `SOTMConstants`. The current value (0.1016m / 4 inches) is a placeholder.
2. Verify stationary shooting still works — with the robot not moving, SOTM should be inactive (speed below `MIN_SPEED_THRESHOLD`) and behavior is identical to before.

## Test Procedure

### Test 1: Stationary Validation

1. Place the robot at a known calibrated position
2. Shoot while stationary
3. Confirm shots hit the target as before — SOTM should not activate below 0.1 m/s

### Test 2: Lateral Compensation

1. Set up about 3-5 meters from the target
2. Drive **sideways** (perpendicular to the target) at ~0.5-1.0 m/s while shooting
3. Observe where the balls land relative to the goal:
   - **Balls miss in direction of travel** → `BALL_VELOCITY_FACTOR` is too low (undercorrecting)
   - **Balls miss opposite direction of travel** → `BALL_VELOCITY_FACTOR` is too high (overcorrecting)
4. Adjust `BALL_VELOCITY_FACTOR` in 0.05 increments and repeat

### Test 3: Range Compensation

1. Drive **toward** the target while shooting
2. Drive **away** from the target while shooting
3. Observe:
   - **Shots fall short while driving toward** → TOF estimate may be too low, try increasing `BALL_VELOCITY_FACTOR`
   - **Shots overshoot while driving toward** → TOF estimate too high, decrease `BALL_VELOCITY_FACTOR`

### Test 4: High-Speed Driving

1. Repeat lateral test at higher speeds (1.5-2.0 m/s)
2. If accuracy degrades significantly at speed, the velocity smoothing or TOF model may need adjustment

## Parameter Reference

| Parameter | Default | Description |
|-----------|---------|-------------|
| `ENABLED` | `true` | Master enable/disable for SOTM |
| `FLYWHEEL_DIAMETER_METERS` | 0.1016 (4") | Physical flywheel wheel diameter — **must be measured** |
| `BALL_VELOCITY_FACTOR` | 0.5 | Fraction of flywheel surface speed that becomes ball horizontal velocity. Accounts for slip, drag, and launch angle |
| `MIN_SPEED_THRESHOLD` | 0.1 m/s | Robot speed below which SOTM is bypassed |
| `MAX_TOF_SECONDS` | 1.0 s | Safety clamp on estimated time-of-flight |
| `TOF_ITERATIONS` | 2 | Number of convergence iterations for virtual target calculation |
| `VELOCITY_SMOOTHING_ALPHA` | 0.3 | Low-pass filter on robot velocity (0 = frozen, 1 = no filtering) |

## Tuning Quick Reference

| Parameter | Increase If... | Decrease If... |
|-----------|----------------|----------------|
| `BALL_VELOCITY_FACTOR` | Balls miss in direction of travel (undercorrecting) | Balls miss opposite of travel direction (overcorrecting) |
| `VELOCITY_SMOOTHING_ALPHA` | Aim feels laggy / slow to respond | Aim is jerky or oscillating |
| `MIN_SPEED_THRESHOLD` | SOTM flickers on/off at very low speed | SOTM doesn't activate soon enough while driving |
| `TOF_ITERATIONS` | Unlikely needed (2 is sufficient) | Want to save CPU cycles (1 is usually fine) |

## How It Works

1. Robot velocity is read from the drivetrain (`ChassisSpeeds`), converted to field-relative, and smoothed
2. If speed > `MIN_SPEED_THRESHOLD`, SOTM activates:
   - **Virtual target** = real target - (robot velocity * TOF)
   - **Virtual position** = robot position + (robot velocity * TOF)
   - Hood angle and flywheel RPM are looked up at the *virtual position* in the calibration grid
   - Turret aims at the *virtual target* from the actual robot position
3. TOF is estimated from distance and flywheel surface speed, then refined over 2 iterations as the virtual target changes the distance

## Disabling SOTM

Set `SOTMConstants.ENABLED = false` in `Constants.java` to completely disable motion compensation. The system will revert to standard stationary aiming at all speeds.
