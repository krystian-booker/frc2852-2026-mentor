// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static class OperatorConstants {
    public static final int kDriverControllerPort = 0;
  }

  public static class FlywheelConstants {
    // CAN IDs
    public static final int kLeaderMotorId = 20;
    public static final int kFollowerMotorId = 21;
    public static final String kCANBus = "canivore";

    // Mechanical
    public static final double kGearRatio = 2.0; // Motor rotations per flywheel rotation

    // PID/Feedforward Gains
    public static final double kS = 0.0;   // Static friction (Amps)
    public static final double kV = 0.12;  // Velocity feedforward (Amps per RPS)
    public static final double kA = 0.01;  // Acceleration feedforward (Amps per RPS/s)
    public static final double kP = 0.5;   // Proportional (Amps per RPS error)
    public static final double kI = 0.0;   // Integral
    public static final double kD = 0.0;   // Derivative

    // Current Limits (v26 API)
    public static final double kSupplyCurrentLimit = 60.0;       // Amps - main limit
    public static final double kSupplyCurrentLowerLimit = 40.0;  // Amps - reduced limit after time
    public static final double kSupplyCurrentLowerTime = 1.0;    // Seconds - time before reducing
    public static final double kStatorCurrentLimit = 120.0;      // Amps

    // Velocity Control
    public static final double kVelocityToleranceRPM = 50.0;     // RPM tolerance for atSetpoint()
    public static final double kSignalUpdateFrequencyHz = 250.0; // Status signal update rate
  }

  public static class HoodConstants {
    // CAN IDs
    public static final int kMotorId = 30;
    public static final int kCANCoderId = 31;
    public static final String kCANBus = "canivore";

    // Mechanical
    public static final double kGearRatio = 15.0; // Motor rotations per hood rotation
    public static final double kMinPositionDegrees = 0.0;
    public static final double kMaxPositionDegrees = 45.0;

    // PID Gains (Slot 0)
    public static final double kS = 0.0;   // Static friction (Amps)
    public static final double kV = 0.0;   // Velocity feedforward (Amps per RPS)
    public static final double kA = 0.0;   // Acceleration feedforward (Amps per RPS/s)
    public static final double kP = 50.0;  // Proportional (Amps per rotation error)
    public static final double kI = 0.0;   // Integral
    public static final double kD = 0.5;   // Derivative

    // Motion Magic
    public static final double kMotionMagicCruiseVelocity = 200.0; // deg/s
    public static final double kMotionMagicAcceleration = 400.0;   // deg/s^2
    public static final double kMotionMagicJerk = 4000.0;          // deg/s^3

    // Current Limits
    public static final double kSupplyCurrentLimit = 40.0;       // Amps - main limit
    public static final double kSupplyCurrentLowerLimit = 30.0;  // Amps - reduced limit after time
    public static final double kSupplyCurrentLowerTime = 1.0;    // Seconds - time before reducing
    public static final double kStatorCurrentLimit = 80.0;       // Amps

    // Position Control
    public static final double kPositionToleranceDegrees = 0.5;  // Degrees tolerance for atPosition()
    public static final double kSignalUpdateFrequencyHz = 250.0; // Status signal update rate

    // CANCoder
    public static final double kCANCoderOffset = 0.0; // Rotations - tune during setup
  }
}
