# Subsystem Tuning Guide (SysId, Feedforward & PID)

This guide takes you from an untuned subsystem (like your Turret, Flywheel, Hood, or IntakeActuator) to a fully tuned system using **SysId** for feedforward characterization, followed by manual adjustments for PID control.

---

## 🏗️ Prerequisites
Before tuning, ensure the following for the subsystem you are tuning:
1. **Current limits are safely set**. (e.g. 80A for your Turret).
2. **Soft limits are correct**. If it's an arm/hood/turret, make sure the limits prevent physical damage.
3. The robot is **on blocks or safe to move**.

---

## Part 1: Getting Data via SysId

SysId handles calculating your Feedforward constants: `kV` (Velocity), `kA` (Acceleration), and `kS` (Static Friction). `kG` (Gravity) is usually calculated manually or via Reca.lc, except for elevators/arms configured for it.

### 1. Enter Test Mode
Your robot codebase is setup to bind SysId routines to the controller **ONLY when the robot is in Test Mode**.
1. Open up the Driver Station.
2. Ensure your active controller is plugged in to Port 0 (Driver Controller).
3. Click **Test** to enable the robot in Test Mode. 
   *(Note: The CTRE `SignalLogger` will automatically start recording when you do this!)*

### 2. Run the 4 SysId Tests
There are four standardized tests you must run for *each* subsystem. You should run them in this order. Let the mechanism return to its starting position (manually or via code if necessary) between each test.

> [!CAUTION]
> These tests will apply raw voltage to the motors. For positional mechanisms with hard stops (Hood, Turret, IntakeActuator), be ready to let go of the button or disable the robot immediately if it approaches the physical limits!

**Test Bindings for your subsystems:**
*(Make sure only the lines for the subsystem you are tuning are uncommented in `RobotContainer.configureTestBindings()`! Right now they are commented out.*)

* **Button A:** Quasistatic Forward (Slowly ramps up voltage forward)
* **Button B:** Quasistatic Reverse (Slowly ramps up voltage backward)
* **Button X:** Dynamic Forward (Applies a step voltage forward instantly)
* **Button Y:** Dynamic Reverse (Applies a step voltage backward instantly)

**Execution Steps:**
1. Enable Test Mode.
2. Hold **A** until the mechanism has moved significantly (or reaches its limit). Release to stop.
3. Move the mechanism back to the start.
4. Hold **B** until it moves significantly. Release to stop.
5. Move back to start.
6. Hold **X** until it moves. Release.
7. Move back to start.
8. Hold **Y** until it moves. Release.
9. **Disable** the robot. 

### 3. Retrieve the Log File
* **For CTRE Motors (Turret, Flywheel, Hood):** 
  The CTRE `SignalLogger` creates `.wpilog` files directly on a USB drive plugged into the roboRIO, or internally on the roboRIO if no USB is present. You can download these via the roboRIO web dashboard or through WinSCP/FileZilla.
* **For REV Motors (IntakeActuator):**
  WPILib's internal data logger will create a WPILog file on the roboRIO. 

---

## Part 2: Analyzing Data in SysId

1. Open WPILib VS Code.
2. Click the WPILib icon in the top right -> **Start Tool** -> **SysId**.
3. In SysId, click the **Open** button next to Data File and select your `.wpilog` file.
4. **Select the correct mechanism:**
   * For CTRE logs: The mechanism name will usually match what we set in code (e.g. `turret-state`). 
   * For REV logs: Look for `intake-actuator-state`.
5. SysId will load the 4 tests. Ensure all 4 graphs look clean (no massive spikes or missing data).

### Analyzer Settings
1. Go to the **Analyzer** tab.
2. Enter the **Measurement Type**:
   * **Flywheel / Intake Roller:** Velocity
   * **Turret / Hood / Intake Actuator / Swerve:** Position
3. **Units:**
   * Set the units to match what your code expects (Rotations or Degrees for position, RPM or RPS for velocity). *Your constants files currently expect Rotations / Amps for CTRE, and Degrees / Volts for REV.*
   * **Important for CTRE:** Since you are using `TorqueCurrentFOC` in `Turret.java`, check if there's an option to output the Feedforward constants relative to Amps! If SysId only gives Volts, you must either switch your control mode to `VoltageOut` or calculate the Volt-to-Amp conversion factor for a Kraken/Falcon.
4. SysId will output `kS`, `kV`, and `kA`. 
   
---

## Part 3: Applying Feedforward & PID

### 1. Apply Feedforward
Take the `kS`, `kV`, and `kA` values from SysId and put them in your `Constants.java` file under the respective subsystem (e.g., `TurretConstants.S`).

**Verify Feedforward:**
1. Set the PID `kP`, `kI`, `kD` values to `0.0`.
2. Deploy the code.
3. Command the mechanism to move (e.g., set the Turret to aim at 90 degrees or Flywheel to 3000 RPM).
4. The mechanism should move *almost* exactly to the target, or track the target velocity very closely, solely under its own predicted physics. If it violently oscillates or doesn't move, your feedforward units are wrong.

### 2. Manual PID Tuning
Once feedforward is doing 90% of the work, use PID to fix the remaining 10% of error.

1. **Tune Proportional (kP):**
   * Increase `kP` slowly (e.g., start at 0.1).
   * Command the system to a setpoint.
   * If it doesn't reach the setpoint fast enough, double `kP`.
   * If it starts oscillating (shaking) violently around the setpoint, cut `kP` in half.
2. **Tune Derivative (kD):** *(Optional, mainly for positional mechanisms like Turrets)*
   * If the mechanism overshoots the target and swings back, increase `kD` slightly to act as a "damper". 
   * Start with `kD` 10x larger than `kP` for CTRE motors (their units are weird) or 0.1x for REV motors.
3. **Tune Integral (kI):** *(Avoid if possible)*
   * `kI` is almost never needed if feedforward is correct. It fixes steady-state error (e.g., a heavy arm drooping slightly below the target).

### 3. Motion Magic (CTRE Only)
For positional mechanisms (Turret, Hood), your code uses CTRE's **Motion Magic**. 
1. `CruiseVelocity`: The max speed you want the mechanism to physically reach (e.g. 1 rotation/s).
2. `Acceleration`: How fast it accelerates to that cruise velocity (e.g. 2 rotations/s²).
3. `Jerk`: How smoothly the acceleration applies. Start with 10x the Acceleration value.

If the mechanism is too snappy, lower the acceleration. If it's too sluggish, raise it.
