# Swerve Drivetrain Tuning Guide

Your swerve drivetrain uses CTRE's **Swerve API v6** and was generated using the **Phoenix Tuner X Swerve Project Generator**. Because of this, tuning Swerve is slightly different than standard subsystems.

You have two primary ways to tune Swerve. The **Recommended Method** is completely UI-driven via Phoenix Tuner X. The **Manual Method** uses the SysId routines currently present in `CommandSwerveDrivetrain.java`.

---

## 🌟 Method 1: Phoenix Tuner X (Highly Recommended)

Phoenix Tuner X has a built-in Swerve Generator and Tuner that natively handles SysId and writes the values directly to your `TunerConstants.java` file. You do not need to mess with log files or WPILib's SysId tool.

### Steps:
1. **Put the Robot on Blocks:** Ensure the wheels are off the ground!
2. Open **Phoenix Tuner X** on your driver station laptop.
3. Connect to your robot (IP or USB).
4. Navigate to the **Swerve** tab on the left sidebar.
5. In the Swerve window, click the **Tuning** tab.
6. **Follow the On-Screen Prompts:**
   * Tuner X will guide you through running Quasistatic and Dynamic tests for both the **Steer** motors and the **Drive (Translation)** motors directly from the UI.
   * *Note:* You must keep the robot enabled while running these tests via Tuner X.
7. **Apply Gains:** Once the tests are finished, Tuner X calculates the `kP`, `kI`, `kD`, `kV`, `kS` values automatically. You can click a button to push these directly to your robot project's `TunerConstants.java` file.
8. Re-deploy your code.

---

## 🛠️ Method 2: Manual SysId (Using Code Bindings)

If you prefer to run SysId manually, your `CommandSwerveDrivetrain.java` has routines built-in for Translation, Steer, and Rotation.

### 1. Code Preparation
1. Open `RobotContainer.java`.
2. In `configureTestBindings()`, uncomment the 4 lines corresponding to the `drivetrain` SysId bindings (near line 296).
   ```java
   RobotModeTriggers.test().and(driverController.a()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kForward));
   // ... etc
   ```
3. Open `CommandSwerveDrivetrain.java`. 
4. Locate the line: `private SysIdRoutine m_sysIdRoutineToApply = m_sysIdRoutineTranslation;` (near line 122).
5. **To tune Drive/Translation:** Leave it as `m_sysIdRoutineTranslation`.
6. **To tune Steer:** Change it to `m_sysIdRoutineSteer`.
7. Turn the robot on and deploy the code.

### 2. Running the Tests
*Same procedure as standard subsystems:*
1. Enable in **Test Mode**.
2. Run Quasistatic Forward (A), Reverse (B).
3. Run Dynamic Forward (X), Reverse (Y).
4. *Important:* Translation tests will make the robot drive across the room. Ensure you have plenty of space! Steer tests are best done on blocks.
5. Pull the `.wpilog` file from the roboRIO via USB or WinSCP.

### 3. Analyzing and Applying
1. Open WPILib SysId.
2. Load the `.wpilog`.
3. Select the mechanism name (e.g. `SysIdTranslation_State` or `SysIdSteer_State`).
4. Apply standard settings for **Velocity** (Drive) or **Position** (Steer).
5. **Where to put the values:** 
   Open `TunerConstants.java`. Look for `steerGains` or `driveGains`. 
   ```java
   private static final Slot0Configs steerGains = new Slot0Configs()
       .withKP(YOUR_P_HERE).withKI(0).withKD(YOUR_D_HERE)
       .withKS(YOUR_S_HERE).withKV(YOUR_V_HERE).withKA(YOUR_A_HERE);
   ```
   Apply the values and re-deploy.
