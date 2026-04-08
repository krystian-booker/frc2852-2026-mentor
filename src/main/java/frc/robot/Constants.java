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
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.RobotBase;
import frc.robot.util.firecontrol.ShooterPhysicsConstants;
import java.util.List;

public final class Constants {

  public static final double SIGNAL_UPDATE_FREQUENCY_HZ = 250.0;
  public static final double loopPeriodSecs = 0.02;
  public static final boolean tuningMode = true; // Set false for competition

  public static final Mode simMode = Mode.SIM;
  public static final Mode currentMode = RobotBase.isReal() ? Mode.REAL : simMode;

  public static enum Mode {
    /** Running on a real robot. */
    REAL,

    /** Running a physics simulator. */
    SIM,

    /** Replaying from a log file. */
    REPLAY
  }

  public static class OperatorConstants {
    public static final int DRIVER_CONTROLLER_PORT = 0;
    public static final int OPERATOR_CONTROLLER_PORT = 1;
  }

  public static class CANIds {
    public static final String CANIVORE = "canivore";

    // Intake
    public static final int INTAKE_ACTUATOR_MOTOR = 13;
    public static final int INTAKE_RIGHT_MOTOR = 14;
    public static final int INTAKE_LEFT_MOTOR = 15;

    // Indexer
    public static final int INDEXER_LEADER_MOTOR = 17;
    public static final int INDEXER_FOLLOWER_ONE_MOTOR = 18;
    public static final int INDEXER_FOLLOWER_TWO_MOTOR = 19;

    // Shooter
    public static final int FLYWHEEL_LEADER_MOTOR = 20;
    public static final int FLYWHEEL_FOLLOWER_MOTOR = 21;
    public static final int TURRET_MOTOR = 22;
    public static final int TURRET_CANCODER = 23;
    public static final int HOOD_MOTOR = 24;

    // PDP
    public static final int PDB = 40;
  }

  public static class FlywheelConstants {
    public static final double GEAR_RATIO = 1.2;

    public static final double MIN_RPM = 1000.0;
    public static final double MAX_RPM = 4700.0;

    public static final double S = 2.1000;
    public static final double V = 0.2203;
    public static final double A = 0.6749;
    public static final double P = 8.0000;
    public static final double I = 0.0000;
    public static final double D = 0.1600;

    // MATlab config - to test
    // public static final double S = 1.4934;
    // public static final double V = 0.0698;
    // public static final double A = 0.8589;
    // public static final double P = 12.6325;
    // public static final double I = 37.9896;
    // public static final double D = 0.0000;

    public static final double SUPPLY_CURRENT_LIMIT = 60.0;
    public static final double SUPPLY_CURRENT_LOWER_LIMIT = 40.0;
    public static final double SUPPLY_CURRENT_LOWER_TIME = 1.0;
    public static final double STATOR_CURRENT_LIMIT = 60.0;

    public static final double VELOCITY_TOLERANCE_RPM = 100.0;
  }

  public static class HoodConstants {
    public static final double GEAR_RATIO = 51.6;
    public static final double MIN_POSITION_DEGREES = 0.0;
    public static final double MAX_POSITION_DEGREES = 25.0;

    /** Actual shot elevation (degrees from horizontal) when hood is at mechanism position 0. */
    public static final double ACTUAL_ANGLE_AT_ZERO_POSITION = 70.0;

    /** Convert mechanism position (0-25) to actual launch elevation (70-45). */
    public static double mechanismToActualAngle(double mechanismDeg) {
      return ACTUAL_ANGLE_AT_ZERO_POSITION - mechanismDeg;
    }

    /** Convert actual launch elevation (70-45) to mechanism position (0-25). */
    public static double actualToMechanismAngle(double actualDeg) {
      return ACTUAL_ANGLE_AT_ZERO_POSITION - actualDeg;
    }

    public static final double S = 3.5;
    public static final double V = 3.8;
    public static final double A = 0.0;
    public static final double G = 0.18;
    public static final double P = 150.0;
    public static final double I = 0.0;
    public static final double D = 1.5;

    public static final double MOTION_MAGIC_CRUISE_VELOCITY = 100.0;
    public static final double MOTION_MAGIC_ACCELERATION = 800.0;
    public static final double MOTION_MAGIC_JERK = 8000.0;

    public static final double SUPPLY_CURRENT_LIMIT = 40.0;
    public static final double SUPPLY_CURRENT_LOWER_LIMIT = 30.0;
    public static final double SUPPLY_CURRENT_LOWER_TIME = 0.5;
    public static final double STATOR_CURRENT_LIMIT = 40.0;

    public static final double POSITION_TOLERANCE_DEGREES = 2;

    public static final double HOMING_VOLTAGE = -1.5;
    public static final double HOMING_STALL_CURRENT_THRESHOLD_AMPS = 8.0;
    public static final double HOMING_STALL_DETECTION_DELAY_SECONDS = 0.25;
    public static final int HOMING_STALL_SAMPLE_COUNT = 5;
    public static final double HOMING_TIMEOUT_SECONDS = 3.0;
  }

  public static class TurretConstants {

    // Turret at physical zero
    public static final double CANCODER_OFFSET = -0.359131;

    // Forward offset - turret pointing straight
    public static final double FORWARD_ENCODER_POSITION_DEGREES = 48.5;

    public static final double GEAR_RATIO = 50.0;
    public static final double ENCODER_MIN_DEGREES = -180.0;
    public static final double ENCODER_MAX_DEGREES = 180.0;
    public static final double MIN_POSITION_DEGREES =
        ENCODER_MIN_DEGREES - FORWARD_ENCODER_POSITION_DEGREES; // -225
    public static final double MAX_POSITION_DEGREES =
        ENCODER_MAX_DEGREES - FORWARD_ENCODER_POSITION_DEGREES; // +135
    public static final double SOFT_LIMIT_BUFFER_DEGREES = 5.0;

    public static final double S = 1.2;
    public static final double V = 4.8862;
    public static final double A = 0.1000;
    public static final double G = 0.0;
    public static final double P = 80.0000;
    public static final double I = 1.0;
    public static final double D = 1.5;

    public static final double MOTION_MAGIC_CRUISE_VELOCITY = 5.0000;
    public static final double MOTION_MAGIC_ACCELERATION = 25.0000;
    public static final double MOTION_MAGIC_JERK = 200.0000;

    public static final double SUPPLY_CURRENT_LIMIT = 60.0;
    public static final double SUPPLY_CURRENT_LOWER_LIMIT = 40.0;
    public static final double SUPPLY_CURRENT_LOWER_TIME = 0.5;
    public static final double STATOR_CURRENT_LIMIT = 60.0;

    public static final double POSITION_TOLERANCE_DEGREES = 1.0;
  }

  public static class TurretAimingConstants {
    // All positions defined for blue alliance; AllianceFlipUtil flips for red
    public static final Translation2d BLUE_TARGET_POSITION = new Translation2d(4.620, 4.035);

    public static final double BLUE_ZONE_MAX_X = 4.625;
    public static final double FIELD_CENTERLINE_Y = 4.035;

    // Non-goal targets (used in neutral/opponent zone)
    public static final Translation2d BLUE_LEFT_TARGET_POSITION = new Translation2d(1.5, 2.5);
    public static final Translation2d BLUE_RIGHT_TARGET_POSITION = new Translation2d(1.5, 6.5);

    // Turret mounting offset from robot center
    public static final double TURRET_OFFSET_X_METERS = Units.inchesToMeters(-4.75);
    public static final double TURRET_OFFSET_Y_METERS = Units.inchesToMeters(-4.5);

    public static final double AIM_TOLERANCE_DEGREES = 2.0;
    public static final double MIN_SHOOTING_DISTANCE_METERS = 0;
    public static final double MAX_SHOOTING_DISTANCE_METERS = 99.0;
  }

  public static class IntakeActuatorConstants {
    public static final int SMART_CURRENT_LIMIT = 40;
    public static final int SECONDARY_CURRENT_LIMIT = 50;
    public static final double EXTEND_DUTY_CYCLE = 1.0;
    public static final double RETRACT_DUTY_CYCLE = -1.0;

    public static final double GEAR_RATIO = 1.0;
    public static final double RETRACTED_POSITION = 0.0;
    public static final double EXTENDED_POSITION = 25.5;
    public static final double RETRACTED_POSITION_AG = 10;
    public static final double POSITION_TOLERANCE_ROTATIONS = 0.25;

    public static final double KP = 3;
    public static final double KI = 0.0;
    public static final double KD = 0.0;
    public static final double MAX_OUTPUT = 1.0;
    public static final double MIN_OUTPUT = -1.0;

    public static final double AGITATE_EXTEND_SECONDS = 0.6;
    public static final double AGITATE_RETRACT_SECONDS = 0.6;

    public static final double STEP_TEST_DUTY_CYCLE = 0.15;
  }

  public static class IntakeConstants {
    public static final double GEAR_RATIO = 1.0;
    public static final double INTAKE_VOLTAGE = 12.0;
    public static final double OUTTAKE_VOLTAGE = -12.0;
    public static final double STATOR_CURRENT_LIMIT = 40.0;
    public static final double SUPPLY_CURRENT_LIMIT = 30.0;
    public static final double SUPPLY_CURRENT_LOWER_LIMIT = 25.0;
    public static final double SUPPLY_CURRENT_LOWER_TIME = 0.75;
  }

  public static class IndexerConstants {
    public static final double GEAR_RATIO = 1.0;
    public static final double FEED_SPEED = 1.0;
    public static final double REVERSE_SPEED = -1.0;
    public static final int SMART_CURRENT_LIMIT = 60;
    public static final int SECONDARY_CURRENT_LIMIT = 80;
    public static final int GROUP_SMART_CURRENT_LIMIT = 40;
    public static final int GROUP_SECONDARY_CURRENT_LIMIT = 60;

    // Jam detection
    public static final double JAM_CURRENT_THRESHOLD_AMPS = 55;
    public static final double JAM_REVERSE_DURATION_SECONDS = 0.5;
    public static final double JAM_COOLDOWN_SECONDS = 1.0;
  }

  public static class QuestNavConstants {
    public static final boolean ENABLED = true;

    // Transform from robot center to Quest headset mounting position
    public static final double QUEST_OFFSET_X_METERS = Units.inchesToMeters(-9.213); // Forward/backward from robot center
    public static final double QUEST_OFFSET_Y_METERS = Units.inchesToMeters(8.250); // Left/right from robot center
    public static final double QUEST_OFFSET_Z_METERS = Units.inchesToMeters(13.303); // Up/down from robot center
    public static final double QUEST_YAW_OFFSET_DEGREES = 180.0; // Rotation around vertical axis
    public static final double QUEST_PITCH_OFFSET_DEGREES = 0.0; // Tilt forward/backward
    public static final double QUEST_ROLL_OFFSET_DEGREES = 0.0; // Tilt left/right

    public static final double STD_DEV_X = 0.02;
    public static final double STD_DEV_Y = 0.02;
    public static final double STD_DEV_THETA = 0.035;
    public static final int RESEED_BUTTON_DIO_PORT = 0;
  }

  public static class BallSimConstants {
    // Robot frame dimensions (with bumpers)
    public static final double ROBOT_WIDTH_M = Units.inchesToMeters(28.25);
    public static final double ROBOT_LENGTH_M = Units.inchesToMeters(28.25);
    public static final double BUMPER_HEIGHT_M = Units.inchesToMeters(7.0);

    // Launcher physics (shared with GenerateShotLUT.java via ShooterPhysicsConstants)
    public static final double SLIP_FACTOR = ShooterPhysicsConstants.SLIP_FACTOR;
    public static final double WHEEL_DIAMETER_M = ShooterPhysicsConstants.WHEEL_DIAMETER_M;
    public static final double EXIT_HEIGHT_M = ShooterPhysicsConstants.EXIT_HEIGHT_M;

    // Launch detection
    public static final double INDEXER_FEEDING_VELOCITY_THRESHOLD_RPS = 1.0;
    public static final double LAUNCH_COOLDOWN_SECONDS = 0.25;

    // Spin imparted to launched ball (fraction of flywheel RPM)
    // Set to 0 for single flywheel + solid hood (no backspin mechanism)
    public static final double BACKSPIN_FRACTION = 0.0;
  }

  public static class DiagnosticConstants {
    public static final String LOG_DIRECTORY = "/home/lvuser/logs/";
    public static final boolean TURRET_LOGGING_DEFAULT_ENABLED = true;
  }

  public static class Vision {
    // Camera configuration record for multi-camera support
    public record CameraConfig(String name, Transform3d robotToCam) {}

    // Left camera
    public static final CameraConfig LEFT_CAMERA =
        new CameraConfig(
            "LEFT_CAMERA",
            new Transform3d(
                new Translation3d(0.0244348, 0.2737866, 0.6747256),
                new Rotation3d(Math.toRadians(0), Math.toRadians(-30), Math.toRadians(-45))));

    // Right camera
    public static final CameraConfig RIGHT_CAMERA =
        new CameraConfig(
            "RIGHT_CAMERA",
            new Transform3d(
                new Translation3d(0.0394716, -0.263906, 0.6851142),
                new Rotation3d(Math.toRadians(0), Math.toRadians(-30), Math.toRadians(45))));

    // Limelight 4
    public static final CameraConfig LIMELIGHT = new CameraConfig("LIMELIGHT",
        new Transform3d(new Translation3d(Units.inchesToMeters(-10.941), Units.inchesToMeters(8.250), Units.inchesToMeters(15.934)),
            new Rotation3d(Math.toRadians(0), Math.toRadians(0), Math.toRadians(180))));

    // All cameras
    public static final List<CameraConfig> CAMERAS = List.of();

    // The layout of the AprilTags on the field
    public static final AprilTagFieldLayout kTagLayout =
        AprilTagFieldLayout.loadField(AprilTagFields.kDefaultField);

    // The standard deviations of our vision estimated poses, which affect
    // correction rate
    public static final Matrix<N3, N1> kSingleTagStdDevs = VecBuilder.fill(4, 4, 8);
    public static final Matrix<N3, N1> kMultiTagStdDevs = VecBuilder.fill(0.5, 0.5, 1);

    // Pose validity timeout - max age in seconds for a pose to be considered valid
    public static final double POSE_VALIDITY_TIMEOUT = 0.5;
  }
}
