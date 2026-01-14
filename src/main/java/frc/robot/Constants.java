package frc.robot;

public final class Constants {
  public static final double SIGNAL_UPDATE_FREQUENCY_HZ = 250.0; // Status signal update rate

  public static class OperatorConstants {
    public static final int DRIVER_CONTROLLER_PORT = 0;
  }

  public static class CANIds {
    public static final String CANIVORE = "canivore";
    public static final int FLYWHEEL_LEADER_MOTOR = 20;
    public static final int FLYWHEEL_FOLLOWER_MOTOR = 21;
    public static final int HOOD_MOTOR = 30;
    public static final int HOOD_CANCODER = 31;
    public static final int TURRET_MOTOR = 40;
    public static final int TURRET_CANCODER = 41;
  }

  public static class FlywheelConstants {
    // Mechanical
    public static final double GEAR_RATIO = 2.0; // Motor rotations per flywheel rotation

    // PID/Feedforward Gains
    public static final double S = 0.0; // Static friction (Amps)
    public static final double V = 0.12; // Velocity feedforward (Amps per RPS)
    public static final double A = 0.01; // Acceleration feedforward (Amps per RPS/s)
    public static final double P = 0.5; // Proportional (Amps per RPS error)
    public static final double I = 0.0; // Integral
    public static final double D = 0.0; // Derivative

    // Current Limits
    public static final double SUPPLY_CURRENT_LIMIT = 60.0; // Amps - main limit
    public static final double SUPPLY_CURRENT_LOWER_LIMIT = 40.0; // Amps - reduced limit after time
    public static final double SUPPLY_CURRENT_LOWER_TIME = 1.0; // Seconds - time before reducing
    public static final double STATOR_CURRENT_LIMIT = 120.0; // Amps

    // Velocity Control
    public static final double VELOCITY_TOLERANCE_RPM = 50.0; // RPM tolerance for atSetpoint()
  }

  public static class HoodConstants {
    // Mechanical
    public static final double GEAR_RATIO = 15.0; // Motor rotations per hood rotation
    public static final double MIN_POSITION_DEGREES = 0.0;
    public static final double MAX_POSITION_DEGREES = 45.0;

    // PID Gains (Slot 0)
    public static final double S = 0.0; // Static friction (Amps)
    public static final double V = 0.0; // Velocity feedforward (Amps per RPS)
    public static final double A = 0.0; // Acceleration feedforward (Amps per RPS/s)
    public static final double P = 50.0; // Proportional (Amps per rotation error)
    public static final double I = 0.0; // Integral
    public static final double D = 0.5; // Derivative

    // Motion Magic
    public static final double MOTION_MAGIC_CRUISE_VELOCITY = 200.0; // deg/s
    public static final double MOTION_MAGIC_ACCELERATION = 400.0; // deg/s^2
    public static final double MOTION_MAGIC_JERK = 4000.0; // deg/s^3

    // Current Limits
    public static final double SUPPLY_CURRENT_LIMIT = 40.0; // Amps - main limit
    public static final double SUPPLY_CURRENT_LOWER_LIMIT = 30.0; // Amps - reduced limit after time
    public static final double SUPPLY_CURRENT_LOWER_TIME = 1.0; // Seconds - time before reducing
    public static final double STATOR_CURRENT_LIMIT = 80.0; // Amps

    // Position Control
    public static final double POSITION_TOLERANCE_DEGREES = 0.5; // Degrees tolerance for atPosition()

    // CANCoder
    public static final double CANCODER_OFFSET = 0.0; // Rotations - tune during setup
  }

  public static class TurretConstants {
    // Mechanical
    public static final double GEAR_RATIO = 50.0; // Motor rotations per turret rotation - TODO: UPDATE THIS
    public static final double MIN_POSITION_DEGREES = 0.0;
    public static final double MAX_POSITION_DEGREES = 360.0;

    // PID Gains (Slot 0)
    public static final double S = 0.0; // Static friction (Amps)
    public static final double V = 0.0; // Velocity feedforward (Amps per RPS)
    public static final double A = 0.0; // Acceleration feedforward (Amps per RPS/s)
    public static final double G = 0.0; // Gravity feedforward (Amps) - may be needed if turret is off-axis
    public static final double P = 100.0; // Proportional (Amps per rotation error)
    public static final double I = 0.0; // Integral
    public static final double D = 1.0; // Derivative

    // Motion Magic
    public static final double MOTION_MAGIC_CRUISE_VELOCITY = 1.0; // Rotations per second
    public static final double MOTION_MAGIC_ACCELERATION = 2.0; // Rotations per second^2
    public static final double MOTION_MAGIC_JERK = 20.0; // Rotations per second^3

    // Current Limits
    public static final double SUPPLY_CURRENT_LIMIT = 40.0; // Amps - main limit
    public static final double SUPPLY_CURRENT_LOWER_LIMIT = 30.0; // Amps - reduced limit after time
    public static final double SUPPLY_CURRENT_LOWER_TIME = 1.0; // Seconds - time before reducing
    public static final double STATOR_CURRENT_LIMIT = 80.0; // Amps

    // Position Control
    public static final double POSITION_TOLERANCE_DEGREES = 1.0; // Degrees tolerance for isAtPosition()

    // CANCoder
    public static final double CANCODER_OFFSET = 0.0; // Rotations - tune during setup to align 0 degrees
  }
}
