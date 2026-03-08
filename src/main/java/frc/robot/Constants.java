package frc.robot;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
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
    public static final double GEAR_RATIO = 2.0; // Motor rotations per flywheel rotation

    // RPM Limits
    public static final double MIN_RPM = 1000.0; // Minimum flywheel RPM for calibration/validation
    public static final double MAX_RPM = 6000.0; // Maximum flywheel RPM for calibration/validation

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
    public static final double GEAR_RATIO = 258.0; // Motor rotations per hood rotation
    public static final double MIN_POSITION_DEGREES = 0.0;
    public static final double MAX_POSITION_DEGREES = 45.0;

    // PID Gains (Slot 0)
    public static final double S = 0.0; // Static friction (Volts)
    public static final double V = 0.0; // Velocity feedforward (Volts per RPS)
    public static final double A = 0.0; // Acceleration feedforward (Volts per RPS/s)
    public static final double P = 10.0; // Proportional (Volts per rotation error) - reduced from 50 for safe testing
    public static final double I = 0.0; // Integral
    public static final double D = 0.5; // Derivative

    // Motion Magic - SLOW for safe testing
    public static final double MOTION_MAGIC_CRUISE_VELOCITY = 36.0; // deg/s (was 200)
    public static final double MOTION_MAGIC_ACCELERATION = 72.0; // deg/s^2 (was 400)
    public static final double MOTION_MAGIC_JERK = 720.0; // deg/s^3 (was 4000)

    // Current Limits - reduced for safe testing
    public static final double SUPPLY_CURRENT_LIMIT = 20.0; // Amps - main limit (was 40)
    public static final double SUPPLY_CURRENT_LOWER_LIMIT = 15.0; // Amps - reduced limit after time (was 30)
    public static final double SUPPLY_CURRENT_LOWER_TIME = 0.5; // Seconds - time before reducing
    public static final double STATOR_CURRENT_LIMIT = 20.0; // Amps (was 80)

    // Position Control
    public static final double POSITION_TOLERANCE_DEGREES = 0.5; // Degrees tolerance for atPosition()
  }

  public static class ClimbConstants {
    // Mechanical
    public static final double GEAR_RATIO = 75.0; // Motor rotations per mechanism rotation - PLACEHOLDER, needs
                                                  // measuring

    // Position setpoints (mechanism rotations)
    public static final double FULLY_DOWN = 0.0;
    public static final double FULL_EXTENSION_POSITION = 10;
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
    public static final double I = 0.0; // Integral
    public static final double D = 0.5; // Derivative

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
    public static final double MIN_POSITION_DEGREES = -180.0;
    public static final double MAX_POSITION_DEGREES = 180.0;
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
    public static final double SUPPLY_CURRENT_LIMIT = 80.0; // Amps - main limit (was 80)
    public static final double SUPPLY_CURRENT_LOWER_LIMIT = 60; // Amps - reduced limit after time (was 60)
    public static final double SUPPLY_CURRENT_LOWER_TIME = 0.5; // Seconds - time before reducing
    public static final double STATOR_CURRENT_LIMIT = 80.0; // Amps (was 80)

    // Position Control
    public static final double POSITION_TOLERANCE_DEGREES = 1.0; // Degrees tolerance for isAtPosition()

    // CANCoder
    public static final double CANCODER_OFFSET = -0.432373;
  }

  public static class TurretAimingConstants {
    // Target positions (meters)
    public static final Translation2d BLUE_TARGET_POSITION = new Translation2d(0.0, 5.55);
    public static final Translation2d RED_TARGET_POSITION = new Translation2d(16.54, 5.55);

    // Turret mounting offset from robot center (if not centered)
    public static final double TURRET_OFFSET_X_METERS = 0.0;
    public static final double TURRET_OFFSET_Y_METERS = 0.0;

    // Turret zero direction relative to robot forward (degrees)
    // 0 means turret 0 degrees points in the same direction as robot forward
    public static final double TURRET_ZERO_ANGLE_DEGREES = 0.0;

    // Aiming tolerance
    public static final double AIM_TOLERANCE_DEGREES = 2.0;

    // Valid shooting range
    public static final double MIN_SHOOTING_DISTANCE_METERS = 1.5;
    public static final double MAX_SHOOTING_DISTANCE_METERS = 10.0;

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
    // String potentiometer (10-turn 3590S-2-103L on RoboRIO analog input)
    public static final int POTENTIOMETER_CHANNEL = 0; // RoboRIO analog input channel
    public static final double POT_FULL_RANGE = 25.0; // Distance units over full pot voltage range - CALIBRATE ON ROBOT
    public static final double POT_OFFSET = -14.562923469654109; // Distance reading at 0V - CALIBRATE ON ROBOT

    // Position limits (distance units matching pot calibration)
    public static final double MIN_POSITION_DISTANCE = 0.0;
    public static final double MAX_POSITION_DISTANCE = 5.3;

    // PID Gains (runs on RoboRIO, output is duty cycle -1 to 1)
    public static final double P = 0.5; // Proportional
    public static final double I = 0.0; // Integral - eliminates steady-state crawl
    public static final double D = 0.0; // Derivative

    // Max output clamp (0.0 to 1.0) - limits motor duty cycle for safe testing
    public static final double MAX_OUTPUT = 1.0; // 15% power - reduced for safe testing

    // Current Limits - reduced for safe testing
    public static final int SMART_CURRENT_LIMIT = 60; // Amps (was 60)
    public static final int SECONDARY_CURRENT_LIMIT = 60; // Amps (was 60)

    // Position Control
    public static final double POSITION_TOLERANCE_DISTANCE = 0.3; // Distance tolerance for atPosition()

    // Current Spike Safety Retraction
    public static final double CURRENT_SPIKE_THRESHOLD_AMPS = 35.0; // Threshold to trigger safety retraction
    public static final double EXTENDED_POSITION_THRESHOLD_DISTANCE = 2.0; // Consider "extended" above this

    // Agitate command (time-based, alternates extend/retract regardless of position reached)
    public static final double AGITATE_EXTEND_SECONDS = 0.2;
    public static final double AGITATE_RETRACT_SECONDS = 0.2;
  }

  public static class IntakeConstants {
    // Mechanical
    public static final double GEAR_RATIO = 1.0; // Motor rotations per roller rotation - UPDATE THIS

    // Preset Speeds (duty cycle -1.0 to 1.0)
    public static final double INTAKE_SPEED = 1.0; // Speed for intaking game pieces
    public static final double OUTTAKE_SPEED = -0.6; // Speed for ejecting game pieces

    // Current Limits
    public static final int SMART_CURRENT_LIMIT = 40; // Amps
    public static final int SECONDARY_CURRENT_LIMIT = 60; // Amps
  }

  public static class ConveyorConstants {
    // Mechanical
    public static final double GEAR_RATIO = 1.0; // Motor rotations per roller rotation - UPDATE THIS

    // Preset Speeds (duty cycle -1.0 to 1.0)
    // Running at max speed for 8 balls/second throughput
    public static final double FEED_SPEED = 1.0; // Full speed for feeding shooter
    public static final double REVERSE_SPEED = -0.5; // Reverse for clearing jams

    // Current Limits - higher limits for sustained high-speed operation
    public static final int SMART_CURRENT_LIMIT = 60; // Amps
    public static final int SECONDARY_CURRENT_LIMIT = 80; // Amps
  }

  public static class QuestNavConstants {
    // Transform from robot center to Quest headset mounting position
    // Measure these values based on where the Quest is mounted on your robot
    public static final double QUEST_OFFSET_X_METERS = 0.0; // Forward/backward from robot center
    public static final double QUEST_OFFSET_Y_METERS = 0.0; // Left/right from robot center
    public static final double QUEST_OFFSET_Z_METERS = 0.0; // Up/down from robot center
    public static final double QUEST_YAW_OFFSET_DEGREES = 0.0; // Rotation around vertical axis
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
    public static final double FIELD_WIDTH_METERS = 8.21;

    /**
     * Grid cell size for UI display (0.5m). This determines the visual grid shown in the webapp. Note: Build-time
     * processing uses 0.25m distance buckets for finer granularity. See GenerateLookupTables.java for details.
     */
    public static final double GRID_CELL_SIZE_METERS = 0.5;
    public static final double CALIBRATION_START_X = 1.0;
    public static final double CALIBRATION_START_Y = 1.0;
    public static final double CALIBRATION_END_X = 15.5;
    public static final double CALIBRATION_END_Y = 7.0;

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

  public static class Vision {
    // Camera configuration record for multi-camera support
    public record CameraConfig(String name, Transform3d robotToCam) {
    }

    // Left camera
    public static final CameraConfig LEFT_CAMERA = new CameraConfig("LEFT_CAMERA",
        new Transform3d(new Translation3d(0.3, 0.25, 0.5), // 30cm fwd, 25cm left, 50cm up
            new Rotation3d(0, Math.toRadians(-15), Math.toRadians(30)) // Pitched down 15°, yawed left 30°
        ));

    // Right camera
    public static final CameraConfig RIGHT_CAMERA = new CameraConfig("RIGHT_CAMERA",
        new Transform3d(new Translation3d(0.3, -0.25, 0.5), // 30cm fwd, 25cm right, 50cm up
            new Rotation3d(0, Math.toRadians(-15), Math.toRadians(-30)) // Pitched down 15°, yawed right 30°
        ));

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

    // PID gains for simple drive-to-target
    public static final double DRIVE_P = 2.0;
    public static final double DRIVE_I = 0.0;
    public static final double DRIVE_D = 0.0;
    public static final double ROTATION_P = 3.0;
    public static final double ROTATION_I = 0.0;
    public static final double ROTATION_D = 0.0;

    // Tolerances for drive-to-target
    public static final double POSITION_TOLERANCE_METERS = 0.05;
    public static final double ROTATION_TOLERANCE_RADIANS = Math.toRadians(2.0);

    // AprilTag target positions - add/modify based on your field positions
    public enum AprilTagTarget {
      EXAMPLE_TAG_1(1, new Pose2d(2.0, 4.0, Rotation2d.fromDegrees(0))),
      EXAMPLE_TAG_2(2, new Pose2d(3.0, 5.0, Rotation2d.fromDegrees(90))),
      EXAMPLE_TAG_3(6, new Pose2d(4.0, 4.0, Rotation2d.fromDegrees(180)));

      private final int tagId;
      private final Pose2d targetPose;

      AprilTagTarget(int tagId, Pose2d targetPose) {
        this.tagId = tagId;
        this.targetPose = targetPose;
      }

      public int getTagId() {
        return tagId;
      }

      public Pose2d getTargetPose() {
        return targetPose;
      }

      public static AprilTagTarget fromTagId(int tagId) {
        for (AprilTagTarget target : values()) {
          if (target.tagId == tagId) {
            return target;
          }
        }
        return null;
      }
    }
  }

}