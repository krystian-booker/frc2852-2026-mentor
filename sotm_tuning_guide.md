# SOTM Testing & Tuning Guide

Now that the `frc-fire-control` solver and `ProjectileSimulator` are integrated, the robot is using purely simulated physics to calculate RPM, Hood Angles, and Time of Flight based on distance. 

Follow this sequence to validate the math and tune it to match your real-world robot physics.

## Phase 1: Stationary Baseline
Before testing moving shots, we must ensure your robot can shoot accurately while standing completely still.

1. **Deploy and Enable**: Open your drivers station, deploy the code, and enable Teleop.
2. **Shoot from Close Range**: Stand about 2 meters away from the hub. Press your shoot button.
3. **Shoot from Long Range**: Move back to roughly 5 meters and shoot again.

### Tuning the Simulator
If your shots are missing while stationary, **DO NOT** try to manually hardcode RPMs. Instead, adjust the physical parameters passed into `ProjectileSimulator` in `TurretAimingCalculator.java`. 

The most sensitive parameters to tweak:
* **`slipFactor` (currently `0.6`)**: This represents how much of your flywheel's surface velocity is actually transferred to the ball. 
  * *If your shots are landing short (too slow):* Decrease `slipFactor`.
  * *If your shots are sailing over (too fast):* Increase `slipFactor`.
* **`exitHeightM` (currently `0.5m`)**: Make sure this matches the exact height the ball leaves the shooter!
* **`flywheelDiameterM` (currently `4" = 0.1016m`)**: Verify this with your mechanical team.
* **`baseLaunchAngleDeg`**: Ensures the hood angle calculations map properly to the trajectory.

> [!TIP]
> **Why Simulation Rocks**
> If you make the ball fly perfectly at 2 meters and perfectly at 5 meters by tweaking `slipFactor`, the physics engine will perfectly interpolate **every distance in between and beyond**. You no longer need to test 15 spots on the field!

## Phase 2: SOTM Validation (Shooting on the Move)
Once the stationary shots are sinking, the simulator has accurately deduced the ball's **Time of Flight (TOF)**, which is the magic number needed to shoot while moving.

1. **Test Lateral Drive**: Drive left/right past the hub at a moderate speed (~1.5 m/s) and hold your shoot button.
2. Observe where the balls hit relative to the center of the hub.

### Troubleshooting SOTM Leads
The solver works by calculating how long the ball is in the air (TOF), multiplying that by the robot's velocity, and aiming behind/ahead (leading) to compensate.

* **Under-leading (Missing "Behind" the robot's movement):**
  * The robot thinks the ball gets there faster than it actually does. 
  * *Fix:* Increase drag (`dragCoeff`) or slightly decrease `slipFactor` while increasing your base RPM.
* **Over-leading (Missing "Ahead" of the robot's movement):**
  * The robot thinks the ball is in the air for longer than it is.
  * *Fix:* Decrease drag (`dragCoeff`) to shorten calculated TOF.

## Phase 3: Rotational Validation
Once lateral SOTM works, you can test chassis rotational compensation.

1. **Test Spin-Shooting**: Stay in place, hold your shoot button, and spin the chassis rapidly with the right stick. 
2. The back-right Turret offset (`launcherOffsetX/Y`) calculates tangential velocity. If the turret correctly locks onto the target while spinning and sinking shots, you're done!

> [!WARNING]
> **Check Odometry Odometry Frequency**
> The `frc-fire-control` solver estimates chassis acceleration by looking at changes in velocity across 20ms cycles. 
> To do this accurately, ensure your Swerve Odometry is updating **at least 50Hz (every 20ms)**. If your swerve code is lagging, your SOTM calculations may "wobble" as the robot speeds up and slows down.
