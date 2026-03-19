package frc.robot;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import java.util.List;

public final class Constants {

  public static final double SIGNAL_UPDATE_FREQUENCY_HZ = 250.0; // Status signal update rate

  public static class OperatorConstants {
    public static final int DRIVER_CONTROLLER_PORT = 0;
    public static final int OPERATOR_CONTROLLER_PORT = 1;
  }

  public static class CANIds {
    public static final String CANIVORE = "canivore";

    // Intake
    public static final int INTAKE_ACTUATOR_MOTOR = 13;
    public static final int INTAKE_ROLLER_MOTOR = 14;

    // Conveyor
    public static final int CONVEYOR_FLOOR_MOTOR = 17;
    public static final int CONVEYOR_TAKE_UP_MOTOR = 18;

    // Shooter
    public static final int FLYWHEEL_LEADER_MOTOR = 20;
    public static final int FLYWHEEL_FOLLOWER_MOTOR = 21;
    public static final int TURRET_MOTOR = 22;
    public static final int TURRET_CANCODER = 23;
    public static final int HOOD_MOTOR = 24;
    public static final int CLIMB_MOTOR = 26;
  }

  public static class FlywheelConstants {
    // Mechanical
    public static final double GEAR_RATIO = 1.2; // Motor rotations per flywheel rotation

    // RPM Limits
    public static final double MIN_RPM = 1000.0; // Minimum flywheel RPM for calibration/validation
    public static final double MAX_RPM = 4700.0; // Maximum flywheel RPM for calibration/validation

    // PID/Feedforward Gains
    public static final double S = 2.1000;
    public static final double V = 0.2203;
    public static final double A = 0.6749;
    public static final double P = 8.0000;
    public static final double I = 0.0000;
    public static final double D = 0.1600;

    // Current Limits
    public static final double SUPPLY_CURRENT_LIMIT = 60.0; // Amps - main limit
    public static final double SUPPLY_CURRENT_LOWER_LIMIT = 40.0; // Amps - reduced limit after time
    public static final double SUPPLY_CURRENT_LOWER_TIME = 1.0; // Seconds - time before reducing
    public static final double STATOR_CURRENT_LIMIT = 80.0; // Amps

    // Velocity Control
    public static final double VELOCITY_TOLERANCE_RPM = 100.0; // RPM tolerance for atSetpoint()
  }

  public static class HoodConstants {
    // Mechanical
    public static final double GEAR_RATIO = 258.0; // Motor rotations per hood rotation
    public static final double MIN_POSITION_DEGREES = 0.0;
    public static final double MAX_POSITION_DEGREES = 25.0;

    // PID Gains (Slot 0)
    public static final double S = 0.4338;
    public static final double V = 19.0191;
    public static final double A = 0.0;
    public static final double G = 0.0352;
    public static final double P = 80.0;
    public static final double I = 0.0;
    public static final double D = 0.5;

    // Motion Magic - max speed (beyond physical limit, motor-limited)
    public static final double MOTION_MAGIC_CRUISE_VELOCITY = 36000.0; // deg/s
    public static final double MOTION_MAGIC_ACCELERATION = 360000.0; // deg/s^2
    public static final double MOTION_MAGIC_JERK = 3600000.0; // deg/s^3

    // Current Limits - reduced for safe testing
    public static final double SUPPLY_CURRENT_LIMIT = 20.0; // Amps - main limit (was 40)
    public static final double SUPPLY_CURRENT_LOWER_LIMIT = 15.0; // Amps - reduced limit after time (was 30)
    public static final double SUPPLY_CURRENT_LOWER_TIME = 0.5; // Seconds - time before reducing
    public static final double STATOR_CURRENT_LIMIT = 20.0; // Amps (was 80)

    // Position Control
    public static final double POSITION_TOLERANCE_DEGREES = 2; // Degrees tolerance for atPosition()

    // Homing (drives hood to reverse hard stop, detects stall, zeroes encoder)
    public static final double HOMING_VOLTAGE = -1.5; // Volts (negative = toward zero stop; exceeds kS=0.4338V)
    public static final double HOMING_STALL_CURRENT_THRESHOLD_AMPS = 8.0; // Stator amps (well under 20A limit, well
                                                                          // above ~2-3A running current)
    public static final double HOMING_STALL_DETECTION_DELAY_SECONDS = 0.25; // Ignore inrush for 250ms after motor start
    public static final int HOMING_STALL_SAMPLE_COUNT = 5; // 5 consecutive cycles above threshold = 100ms confirmed
                                                           // stall
    public static final double HOMING_TIMEOUT_SECONDS = 3.0; // Safety timeout
  }

  public static class ClimbConstants {
    // Mechanical
    public static final double GEAR_RATIO = 5; // Motor rotations per mechanism rotation - PLACEHOLDER, needs
                                               // measuring

    // Position setpoints (mechanism rotations)
    public static final double FULLY_DOWN = 0.0;
    public static final double FULL_EXTENSION_POSITION = 10.238;
    public static final double CLIMB_LIFT_POSITION = 3.0;

    // Soft limits (mechanism rotations)
    public static final double MIN_POSITION = 0.0;
    public static final double MAX_POSITION = 10.0;

    // PID Gains (Slot 0) - voltage-based
    public static final double S = 0.0; // Static friction (Volts)
    public static final double V = 2.0; // Velocity feedforward (Volts per RPS) - needs tuning
    public static final double A = 0.0; // Acceleration feedforward (Volts per RPS/s)
    public static final double G = 0.0; // Gravity feedforward (Volts)
    public static final double P = 80.0; // Proportional (Volts per rotation error)
    public static final double I = 0.0;
    public static final double D = 0.5;

    // Motion Magic - slow for safety
    public static final double MOTION_MAGIC_CRUISE_VELOCITY = 5.0; // rot/s
    public static final double MOTION_MAGIC_ACCELERATION = 10.0; // rot/s^2
    public static final double MOTION_MAGIC_JERK = 100.0; // rot/s^3

    // Current Limits - high for lifting robot weight
    public static final double SUPPLY_CURRENT_LIMIT = 60.0; // Amps - main limit
    public static final double SUPPLY_CURRENT_LOWER_LIMIT = 40.0; // Amps - reduced limit after time
    public static final double SUPPLY_CURRENT_LOWER_TIME = 1.0; // Seconds - time before reducing
    public static final double STATOR_CURRENT_LIMIT = 120.0; // Amps

    // Position Control
    public static final double POSITION_TOLERANCE = 0.25; // Rotations tolerance for atPosition()
  }

  public static class TurretConstants {
    // Mechanical
    public static final double GEAR_RATIO = 50.0; // Motor rotations per turret rotation

    // Forward offset: encoder reading (degrees) when turret points robot-forward.
    public static final double FORWARD_ENCODER_POSITION_DEGREES = 48.5;

    // Physical encoder limits (used for firmware soft limits — do not change without re-verifying travel)
    public static final double ENCODER_MIN_DEGREES = -180.0;
    public static final double ENCODER_MAX_DEGREES = 180.0;

    // Aiming-relative limits (0 = forward): encoder limits shifted by forward offset
    public static final double MIN_POSITION_DEGREES = ENCODER_MIN_DEGREES - FORWARD_ENCODER_POSITION_DEGREES; // -225
    public static final double MAX_POSITION_DEGREES = ENCODER_MAX_DEGREES - FORWARD_ENCODER_POSITION_DEGREES; // +135

    public static final double SOFT_LIMIT_BUFFER_DEGREES = 5.0; // Extra degrees beyond app range for overshoot recovery

    // PID Gains (Slot 0) - voltage-based, mechanism rotations (50:1 gear ratio)
    public static final double S = 0.4800;
    public static final double V = 4.8862;
    public static final double A = 0.1000;
    public static final double G = 0.0;
    public static final double P = 80.0000;
    public static final double I = 0.0;
    public static final double D = 0.0000;
    public static final double MOTION_MAGIC_CRUISE_VELOCITY = 5.0000;
    public static final double MOTION_MAGIC_ACCELERATION = 25.0000;
    public static final double MOTION_MAGIC_JERK = 200.0000;

    // Current Limits - reduced for safe testing
    public static final double SUPPLY_CURRENT_LIMIT = 20.0; // Amps - main limit (was 80)
    public static final double SUPPLY_CURRENT_LOWER_LIMIT = 60; // Amps - reduced limit after time (was 60)
    public static final double SUPPLY_CURRENT_LOWER_TIME = 0.5; // Seconds - time before reducing
    public static final double STATOR_CURRENT_LIMIT = 40.0; // Amps (was 80)

    // Position Control
    public static final double POSITION_TOLERANCE_DEGREES = 1.0; // Degrees tolerance for isAtPosition()

    // CANCoder
    public static final double CANCODER_OFFSET = -0.359131;
  }

  public static class TurretAimingConstants {
    // Target positions (meters) - alliance goals
    public static final Translation2d BLUE_TARGET_POSITION = new Translation2d(4.625, 4.040);
    public static final Translation2d RED_TARGET_POSITION = new Translation2d(11.915, 4.040);

    // Zone boundaries (X axis, meters)
    public static final double BLUE_ZONE_MAX_X = 4.625;
    public static final double RED_ZONE_MIN_X = 11.905;

    // Field Y centerline for left/right target selection
    public static final double FIELD_CENTERLINE_Y = 4.035;

    // Non-goal targets per alliance (used in neutral/opponent zone)
    public static final Translation2d BLUE_LEFT_TARGET_POSITION = new Translation2d(1.5, 2.5);
    public static final Translation2d BLUE_RIGHT_TARGET_POSITION = new Translation2d(1.5, 6.5);
    public static final Translation2d RED_LEFT_TARGET_POSITION = new Translation2d(15.040, 2.5);
    public static final Translation2d RED_RIGHT_TARGET_POSITION = new Translation2d(15.040, 6.5);

    // Turret mounting offset from robot center (if not centered)
    public static final double TURRET_OFFSET_X_METERS = -4.719 * 0.0254; // inches to meters
    public static final double TURRET_OFFSET_Y_METERS = -5.250 * 0.0254; // inches to meters

    // Aiming tolerance
    public static final double AIM_TOLERANCE_DEGREES = 2.0;

    // Valid shooting range
    public static final double MIN_SHOOTING_DISTANCE_METERS = 0;
    public static final double MAX_SHOOTING_DISTANCE_METERS = 99.0;

    // Placeholder lookup tables (distance in meters -> value)
    // Format: { {distance1, value1}, {distance2, value2}, ... }
    // Hood angle lookup table (distance -> hood angle in degrees)
    public static final double[][] HOOD_LOOKUP_TABLE = { { 1.5, 15.0 }, { 3.0, 25.0 }, { 5.0, 35.0 }, { 7.0, 40.0 },
        { 10.0, 45.0 } };

    // Flywheel RPM lookup table (distance -> RPM)
    public static final double[][] FLYWHEEL_LOOKUP_TABLE = { { 1.5, 3000.0 }, { 3.0, 3500.0 }, { 5.0, 4000.0 },
        { 7.0, 4500.0 }, { 10.0, 5000.0 } };
  }

  public static class IntakeActuatorConstants {
    // Current Limits
    public static final int SMART_CURRENT_LIMIT = 30; // Amps
    public static final int SECONDARY_CURRENT_LIMIT = 60; // Amps

    // Open-loop duty cycles
    public static final double EXTEND_DUTY_CYCLE = 0.5;
    public static final double RETRACT_DUTY_CYCLE = -0.5;

    // Through Bore Encoder (external encoder port on SparkFlex)
    public static final int ENCODER_COUNTS_PER_REV = 8192;
    public static final double GEAR_RATIO = 1.0;
    public static final double RETRACTED_POSITION_ROTATIONS = 0.0;
    public static final double EXTENDED_POSITION_ROTATIONS = 5.0; // PLACEHOLDER - measure on robot
    public static final double POSITION_TOLERANCE_ROTATIONS = 0.25;

    // Closed-loop position PID
    public static final double KP = 0.1;
    public static final double KI = 0.0;
    public static final double KD = 0.0;
    public static final double MAX_OUTPUT = 0.5;
    public static final double MIN_OUTPUT = -0.5;

    // Agitate command (time-based, alternates extend/retract regardless of position reached)
    public static final double AGITATE_EXTEND_SECONDS = 0.4;
    public static final double AGITATE_RETRACT_SECONDS = 0.4;
  }

  public static class IntakeConstants {
    // Mechanical
    public static final double GEAR_RATIO = 1.0; // Motor rotations per roller rotation - UPDATE THIS

    // Preset Speeds (duty cycle -1.0 to 1.0)
    public static final double INTAKE_SPEED = 1.0; // Speed for intaking game pieces
    public static final double OUTTAKE_SPEED = -1.0; // Speed for ejecting game pieces

    // Current Limits
    public static final int SMART_CURRENT_LIMIT = 30; // Amps
    public static final int SECONDARY_CURRENT_LIMIT = 80; // Amps
  }

  public static class ConveyorConstants {
    // Mechanical
    public static final double GEAR_RATIO = 1.0; // Motor rotations per roller rotation - UPDATE THIS

    // Preset Speeds (duty cycle -1.0 to 1.0)
    // Running at max speed for 8 balls/second throughput
    public static final double FEED_SPEED = 0.7; // Full speed for feeding shooter
    public static final double REVERSE_SPEED = -0.5; // Reverse for clearing jams

    // Current Limits - higher limits for sustained high-speed operation
    public static final int SMART_CURRENT_LIMIT = 30; // Amps
    public static final int SECONDARY_CURRENT_LIMIT = 80; // Amps
  }

  public static class QuestNavConstants {
    // Transform from robot center to Quest headset mounting position
    // Measure these values based on where the Quest is mounted on your robot
    public static final double QUEST_OFFSET_X_METERS = -0.14605; // Forward/backward from robot center
    public static final double QUEST_OFFSET_Y_METERS = 0.2099564; // Left/right from robot center
    public static final double QUEST_OFFSET_Z_METERS = 0.4172712; // Up/down from robot center
    public static final double QUEST_YAW_OFFSET_DEGREES = 90.0; // Rotation around vertical axis
    public static final double QUEST_PITCH_OFFSET_DEGREES = 0.0; // Tilt forward/backward
    public static final double QUEST_ROLL_OFFSET_DEGREES = 0.0; // Tilt left/right

    // Standard deviations for pose estimation (meters, meters, radians)
    // Lower values = trust QuestNav more, higher = trust wheel odometry more
    public static final double STD_DEV_X = 0.02;
    public static final double STD_DEV_Y = 0.02;
    public static final double STD_DEV_THETA = 0.035;

    // Movement threshold for detecting if robot was moved while disabled (meters)
    // If robot moves more than this distance from seeded position, re-seeding is required
    public static final double SEEDING_MOVEMENT_THRESHOLD_METERS = 0.1;

    // Polling interval for seeding checks while disabled (seconds)
    public static final double SEEDING_POLL_INTERVAL_SECONDS = 1.0;
  }

  public static class BlinkinConstants {
    public static final int PWM_PORT = 0;
  }

  /**
   * Constants for the turret calibration system.
   *
   * <p>
   * <strong>Grid vs Bucket Spacing Design:</strong> The calibration system uses two different spacing granularities by
   * design:
   *
   * <ul>
   * <li><strong>UI Grid (0.5m cell size):</strong> Used by the webapp to display robot position and track which cells
   * have been calibrated. The larger spacing makes the grid visually manageable and provides clear guidance for where
   * to position the robot.</li>
   * <li><strong>Build-time Distance Buckets (0.25m):</strong> Used by GenerateLookupTables.java when processing
   * calibration data. The finer granularity allows multiple samples taken within the same UI grid cell to be grouped
   * into different distance buckets, providing more precise interpolation data.</li>
   * </ul>
   *
   * <p>
   * This is intentional: a single UI grid cell may contain multiple calibration points at slightly different distances,
   * and the build-time processing preserves this detail for better interpolation accuracy.
   */
  public static class CalibrationConstants {
    // Field dimensions
    public static final double FIELD_LENGTH_METERS = 16.54;
    public static final double FIELD_WIDTH_METERS = 8.07;

    /**
     * Grid cell size for UI display (0.5m). This determines the visual grid shown in the webapp. Note: Build-time
     * processing uses 0.25m distance buckets for finer granularity. See GenerateLookupTables.java for details.
     */
    public static final double GRID_CELL_SIZE_METERS = 0.5;
    public static final double CALIBRATION_START_X = 0.0;
    public static final double CALIBRATION_START_Y = 0.0;
    public static final double CALIBRATION_END_X = FIELD_LENGTH_METERS;
    public static final double CALIBRATION_END_Y = FIELD_WIDTH_METERS;

    // Calculated grid size (30 columns x 13 rows = 390 cells)
    public static final int GRID_COLS = (int) ((CALIBRATION_END_X - CALIBRATION_START_X) / GRID_CELL_SIZE_METERS) + 1;
    public static final int GRID_ROWS = (int) ((CALIBRATION_END_Y - CALIBRATION_START_Y) / GRID_CELL_SIZE_METERS) + 1;

    // Position tolerance for grid cell detection
    public static final double POSITION_TOLERANCE_METERS = 0.15;

    // Default values for inputs
    public static final double DEFAULT_HOOD_ANGLE = 15.0;
    public static final double DEFAULT_FLYWHEEL_RPM = 3000.0;

    // Calibration file path on roboRIO
    public static final String CALIBRATION_FILE_PATH = "/home/lvuser/deploy/calibration/turret_calibration_data.csv";
  }

  public static class AutoConstants {
    public static final double PATHFINDING_MAX_VELOCITY = 1.0; // m/s
    public static final double PATHFINDING_MAX_ACCELERATION = 1.0; // m/s^2
    public static final double PATHFINDING_MAX_ANGULAR_VELOCITY = Math.toRadians(540.0); // rad/s
    public static final double PATHFINDING_MAX_ANGULAR_ACCELERATION = Math.toRadians(720.0); // rad/s^2
    public static final double PATHFINDING_TIMEOUT_SECONDS = 3.0; // Give up pathfinding after this time
  }

  public static class Vision {
    // Camera configuration record for multi-camera support
    public record CameraConfig(String name, Transform3d robotToCam) {
    }

    // Left camera
    public static final CameraConfig LEFT_CAMERA = new CameraConfig("LEFT_CAMERA",
        new Transform3d(new Translation3d(0.0244348, 0.2737866, 0.6747256),
            new Rotation3d(Math.toRadians(0), Math.toRadians(-30), Math.toRadians(-45))));

    // Right camera
    public static final CameraConfig RIGHT_CAMERA = new CameraConfig("RIGHT_CAMERA",
        new Transform3d(new Translation3d(0.0394716, -0.263906, 0.6851142),
            new Rotation3d(Math.toRadians(0), Math.toRadians(-30), Math.toRadians(45))));

    // Limelight 4
    public static final CameraConfig LIMELIGHT = new CameraConfig("LIMELIGHT",
        new Transform3d(new Translation3d(-0.1651, 0.2421636, 0.5263896),
            new Rotation3d(Math.toRadians(0), Math.toRadians(0), Math.toRadians(90))));

    // All cameras
    public static final List<CameraConfig> CAMERAS = List.of(LEFT_CAMERA, RIGHT_CAMERA);

    // The layout of the AprilTags on the field
    public static final AprilTagFieldLayout kTagLayout = AprilTagFieldLayout.loadField(AprilTagFields.kDefaultField);

    // The standard deviations of our vision estimated poses, which affect correction rate
    // (Fake values. Experiment and determine estimation noise on an actual robot.)
    public static final Matrix<N3, N1> kSingleTagStdDevs = VecBuilder.fill(4, 4, 8);
    public static final Matrix<N3, N1> kMultiTagStdDevs = VecBuilder.fill(0.5, 0.5, 1);

    // Pose validity timeout - max age in seconds for a pose to be considered valid
    public static final double POSE_VALIDITY_TIMEOUT = 0.5;

  }

}