# Shooter Calibration & Shoot-on-the-Move Guide

This replaces the old 578-cell lookup grid, the calibration webapp, and
`SOTM_TUNING.md`. The shooter is now driven by two small **distance-keyed shot
maps** plus a physics-based shoot-on-the-move (SOTM) solver.

## How it works

Because the turret always points at the target, the correct hood angle and
flywheel RPM depend only on **horizontal distance to the target** — not on
where the robot is on the field. So instead of calibrating 578 grid cells, you
calibrate **4–8 distances** and everything in between is interpolated:

- **Hub map** — used inside your scoring zone, shooting into the hub (rim at 6 ft).
- **Lob map** — used everywhere else, throwing balls toward your scoring zone.

Each calibration point is `(distance, hood angle, flywheel RPM)`. Time of
flight is **derived automatically** from the calibrated arc: a ball launched at
elevation θ that passes through the rim at distance *d* and height rise *Δh*
must satisfy `Δh = d·tanθ − ½gt²`, which gives `t = √(2(d·tanθ − Δh)/g)` —
no chronograph needed. That flight time is what the SOTM solver uses to lead
the target, so improving the shot map automatically improves moving shots.

Maps are saved to `/home/lvuser/shotmaps/{hub,lob}.csv` on the roboRIO —
**they survive code deploys**. In-code defaults (`ShooterModelConstants`) are
used until the first calibration is saved. After a good session, press
`Calibration/PrintMaps` and paste the console output into the defaults so the
calibration is also in git.

## Calibration procedure (~15 minutes)

Prerequisites: pose estimation healthy (QuestNav seeded / tags visible), hood
homed, and game pieces in match condition (worn foam balls fly differently
than fresh ones). Measure and set `SHOOTER_EXIT_HEIGHT_METERS` in
`Constants.ShooterModelConstants` once — it's the height of the ball exit
above the carpet.

1. **Enable Test mode.** The turret keeps auto-aiming at the target. The hood
   and flywheel follow the dashboard values `Calibration/HoodAngle` and
   `Calibration/FlywheelRPM`.
2. Park at a distance (start ~1.5 m from the hub). Watch
   `Calibration/DistanceMeters` and `Calibration/ActiveMap` (HUB or LOB —
   picked automatically from your field position).
3. Feed balls with the **driver right trigger**. Adjust hood/RPM on the
   dashboard until shots land dead center consistently.
4. Press **driver A** (or the `Calibration/Record` dashboard button). The
   point is keyed to the current distance and saved immediately.
   **Driver B** undoes the last point if you fat-finger one.
5. Move back ~1 m and repeat. Cover the hub range in 4–6 points (e.g. 1.5,
   2.5, 3.5, 4.5, 5.5 m). Recording within 0.15 m of an existing point
   replaces it, so re-tuning a spot is just record-again.
6. Drive outside the scoring zone and repeat for the lob map at 3–4 distances.
   Lob shots only need to land in the zone, so be quick about it.

Tip: `Calibration/MapHoodAtDistance` / `MapRPMAtDistance` show what the
current map already predicts where you're standing — if shots with those
values already land, skip that distance.

## Shoot-on-the-move: what changed and why it missed before

The old code led the target by `distance / 12.0 m/s` — a constant "average
ball speed". Three real effects broke it:

1. **Flight time is not linear in distance.** A close, steep shot (hood 0 →
   70° launch) actually flies *longer* than a mid-range flatter shot. Using
   12 m/s underestimated close-range lead badly. Now ToF comes from the
   calibrated arc geometry per distance.
2. **The ball may not inherit all of the robot's velocity.** Feeder friction
   and ball slip can shed part of the chassis velocity at release. 254's 2022
   implementation assumed 100% and won with it, so that's the default — but
   it's now tunable (`Shooter/SOTM/VelocityInheritance`) instead of baked in.
3. **The solution was solved once, not iterated, and used a stale snapshot.**
   Lead changes distance, distance changes flight time, which changes lead —
   the solver now fixed-point iterates 3×, and hood/RPM/turret angle always
   come from the same fresh solve (the old code could serve the hood a stale
   time-of-flight when the operator touched the override stick).

Also fixed: **feed gating**. Balls used to feed as soon as the flywheel was at
speed — even if the hood or turret was still moving. `ShootCommand` now waits
for flywheel + hood + turret all on target.

## SOTM tuning knobs (live on the dashboard, no redeploy)

| Key | Default | Meaning | Symptom → fix |
|---|---|---|---|
| `Shooter/SOTM/VelocityInheritance` | 1.0 | Fraction of chassis velocity the ball keeps | Moving shots miss **ahead** of the hub (in the direction you're driving) → lower it. Miss **behind** → raise it. |
| `Shooter/SOTM/TofScale` | 1.05 | Drag correction on vacuum flight time | All moving shots under-led regardless of direction → raise slightly (foam balls have real drag). |
| `Shooter/SOTM/ReleaseLookaheadSecs` | 0.10 | How far ahead the robot pose is projected to ball release | Misses only show up at high speed and scale with speed → raise in 0.05 steps. |
| `SOTM_ENABLED` (Constants) | true | Master switch | Set false to verify stationary accuracy is unaffected. |
| `SOTM_MAX_LEAD_METERS` (Constants) | 1.5 | Safety clamp on lead | Raise only after everything else is trusted. |

### Tuning order

1. **Stationary first.** SOTM can't fix a bad shot map. Verify stationary
   accuracy at 3–4 distances; recalibrate points if needed.
2. **Verify no regression:** stationary shots with SOTM enabled must be
   identical (lead is zero at zero velocity by construction).
3. **Drive perpendicular** (strafe past the hub at ~1.5 m/s) and shoot
   continuously. Perpendicular motion isolates `VelocityInheritance`: adjust
   until shots stop trailing/leading sideways.
4. **Drive toward/away** from the hub and shoot. Radial motion exercises the
   effective-distance math and `TofScale`: short while approaching + long
   while retreating → raise `TofScale`; the reverse → lower it.
5. **Increase speed** and fix residual speed-proportional error with
   `ReleaseLookaheadSecs`.
6. When happy, copy the dashboard values into the `_DEFAULT` constants in
   `ShooterModelConstants` so they're permanent.

### Turret feedforward

The turret velocity feedforward now includes both counter-rotation against
chassis yaw *and* the bearing change from translation (strafing past the hub
makes the turret slew even with zero yaw rate). Same sign convention as
before: if the turret drifts *with* the chassis when spinning in place, flip
the sign where `turretVelocityFFDegreesPerSecond` is passed in
`RobotContainer` (turret default command).

## Consistency checklist (do these before blaming the math)

Research from top 2022 hub-shooter teams says shot-to-shot *mechanical*
consistency dominates accuracy — more than trajectory modeling:

- **Calibrate flat arcs.** Aim for trajectories that just clear the hub rim
  rather than high lobs: less drag sensitivity and far fewer bounce-outs
  (6328's approach).
- **Make the flywheel dip identical every shot.** Each ball drains flywheel
  energy as it accelerates to surface speed. 6328 added a trapezoidal
  velocity ramp (accel + jerk limits) tuned so the dip is the same shot after
  shot. Phoenix 6 equivalent: switch `Flywheel` from
  `VelocityTorqueCurrentFOC` to `MotionMagicVelocityTorqueCurrentFOC` with
  `MotionMagicAcceleration`/`MotionMagicJerk` configured — worth trying if
  rapid-fire shots 2 and 3 land short.
- **Watch ball wear.** Worn/soft foam balls fly measurably differently from
  fresh ones; calibrate with match-condition balls.
- **Hood backlash**: always approach hood setpoints from the same direction if
  drift is observed (Motion Magic already helps here).

## Telemetry to watch

- `Shoot/DistanceMeters`, `Shoot/TimeOfFlight`, `Shoot/IsHubShot` — the live solve
- `Shoot/TargetRPM` / `Shoot/TargetHoodAngle` vs. actuals (shot logger CSV)
- `Turret/AimSetpointDeg`, `Turret/AimVelocityFF`
- `Calibration/*` — everything for calibration mode

## What was removed

- `webapp/` (Vue calibration app), `tools/GenerateLookupTables.java`, the
  build-time codegen in `build.gradle`, `generated/TurretLookupTables.java`,
  `TurretCalibrationCommand`, `recalculate_turret_table.py`, and the 578-row
  CSV. The old grid's knowledge was distilled into the default points in
  `ShooterModelConstants` (median hood/RPM per 0.5 m distance bucket).
