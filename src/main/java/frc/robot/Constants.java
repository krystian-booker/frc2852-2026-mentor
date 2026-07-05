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

import java.util.List;

public final class Constants {

  public static final double SIGNAL_UPDATE_FREQUENCY_HZ = 250.0;

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

    /**
     * Actual shot elevation (degrees from horizontal) when hood is at mechanism
     * position 0.
     */
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
    public static final double CANCODER_OFFSET = 0.005615;

    // Forward offset - turret pointing straight
    public static final double FORWARD_ENCODER_POSITION_DEGREES = 0;

    public static final double GEAR_RATIO = 50.0;
    public static final double ENCODER_MIN_DEGREES = -180.0;
    public static final double ENCODER_MAX_DEGREES = 180.0;
    public static final double MIN_POSITION_DEGREES = ENCODER_MIN_DEGREES - FORWARD_ENCODER_POSITION_DEGREES; // -180
    public static final double MAX_POSITION_DEGREES = ENCODER_MAX_DEGREES - FORWARD_ENCODER_POSITION_DEGREES; // +180
    public static final double SOFT_LIMIT_BUFFER_DEGREES = 5.0;

    /**
     * The turret range spans exactly 360°, so a target bearing sitting right on
     * the ±180° seam flips between the two range limits with the tiniest pose
     * noise — and every flip commands a full-revolution unwind. A new setpoint
     * whose physical bearing change is smaller than this is ignored when it
     * would command such an unwind.
     */
    public static final double WRAP_HYSTERESIS_DEGREES = 10.0;

    public static final double S = 1.2;
    public static final double V = 4.8862;
    public static final double A = 0.1000;
    public static final double G = 0.0;
    public static final double P = 100.0000;
    public static final double I = 1.0;
    public static final double D = 1.5;

    public static final double MOTION_MAGIC_CRUISE_VELOCITY = 5.0000;
    public static final double MOTION_MAGIC_ACCELERATION = 25.0000;
    public static final double MOTION_MAGIC_JERK = 200.0000;

    public static final double SUPPLY_CURRENT_LIMIT = 50.0;
    public static final double SUPPLY_CURRENT_LOWER_LIMIT = 40.0;
    public static final double SUPPLY_CURRENT_LOWER_TIME = 0.5;
    public static final double STATOR_CURRENT_LIMIT = 50.0;

    public static final double POSITION_TOLERANCE_DEGREES = 1.0;
  }

  public static class TurretAimingConstants {
    public static final Translation2d BLUE_TARGET_POSITION = new Translation2d(4.625, 4.040);
    public static final Translation2d RED_TARGET_POSITION = new Translation2d(11.915, 4.040);

    public static final double BLUE_ZONE_MAX_X = 4.625;
    public static final double RED_ZONE_MIN_X = 11.905;
    public static final double FIELD_CENTERLINE_Y = 4.035;

    /**
     * Once inside the scoring zone, the robot must travel this far back past
     * the boundary before the target switches to the lob shot. Without the
     * band, pose noise at the line flips the turret between the hub and lob
     * targets every loop.
     */
    public static final double ZONE_HYSTERESIS_METERS = 0.30;
    /** Same idea for the left/right lob target split at the centerline. */
    public static final double CENTERLINE_HYSTERESIS_METERS = 0.30;

    // Non-goal targets per alliance (used in neutral/opponent zone)
    public static final Translation2d BLUE_LEFT_TARGET_POSITION = new Translation2d(1.5, 2.5);
    public static final Translation2d BLUE_RIGHT_TARGET_POSITION = new Translation2d(1.5, 6.5);
    public static final Translation2d RED_LEFT_TARGET_POSITION = new Translation2d(15.040, 2.5);
    public static final Translation2d RED_RIGHT_TARGET_POSITION = new Translation2d(15.040, 6.5);

    // Turret mounting offset from robot center
    public static final double TURRET_OFFSET_X_METERS = Units.inchesToMeters(-4.75);
    public static final double TURRET_OFFSET_Y_METERS = Units.inchesToMeters(-4.5);

    public static final double AIM_TOLERANCE_DEGREES = 2.0;
    public static final double MIN_SHOOTING_DISTANCE_METERS = 0;
    public static final double MAX_SHOOTING_DISTANCE_METERS = 99.0;

    // SOTM (Shoot on the Move) constants
    public static final boolean SOTM_ENABLED = true;
    public static final double SOTM_MAX_LEAD_METERS = 1.5; // Safety clamp on virtual target offset
  }

  public static class IntakeActuatorConstants {
    public static final int SMART_CURRENT_LIMIT = 60;
    public static final int SECONDARY_CURRENT_LIMIT = 80;
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
    public static final int SMART_CURRENT_LIMIT = 80;
    public static final int SECONDARY_CURRENT_LIMIT = 120;
    public static final int GROUP_SMART_CURRENT_LIMIT = 80;
    public static final int GROUP_SECONDARY_CURRENT_LIMIT = 120;
  }

  /**
   * Physical model and calibration defaults for the distance-based shot maps.
   * See SHOOTER_TUNING.md for the calibration procedure.
   */
  public static class ShooterModelConstants {
    /** Height of the hub rim opening above the carpet (6 ft). */
    public static final double HUB_RIM_HEIGHT_METERS = 1.8288;
    /** Height of the ball exit above the carpet. TODO: measure on the robot. */
    public static final double SHOOTER_EXIT_HEIGHT_METERS = 0.60;
    /** Lob shots land on the carpet. */
    public static final double LOB_LANDING_HEIGHT_METERS = 0.0;
    public static final double GRAVITY = 9.80665;

    /** ToF fallback when ballistic geometry is degenerate or a map is empty. */
    public static final double FALLBACK_BALL_SPEED_MPS = 12.0;
    /** Re-recording within this distance replaces the previous point. */
    public static final double POINT_MERGE_DISTANCE_METERS = 0.15;

    /** Values used if a shot map has no points at all. */
    public static final double EMPTY_MAP_HOOD_DEGREES = 15.0;
    public static final double EMPTY_MAP_FLYWHEEL_RPM = 3000.0;

    /** Fixed-point iterations of the SOTM time-of-flight solve. */
    public static final int SOTM_ITERATIONS = 3;

    // Defaults for the live dashboard tunables under Shooter/SOTM/*
    /**
     * Fraction of chassis velocity the ball keeps at release (0-1). 254's 2022
     * reference implementation assumed 1.0; tune down if moving shots miss in
     * the direction of travel (feeder friction / ball slip).
     */
    public static final double SOTM_VELOCITY_INHERITANCE_DEFAULT = 1.0;
    /** Multiplier on vacuum time-of-flight to account for drag on the foam ball. */
    public static final double SOTM_TOF_SCALE_DEFAULT = 1.05;
    /** Seconds to project the robot pose forward for release latency. */
    public static final double SOTM_RELEASE_LOOKAHEAD_SECONDS_DEFAULT = 0.10;

    /**
     * Directory (under the operating dir, /home/lvuser on the roboRIO) where
     * calibrated shot maps persist across deploys.
     */
    public static final String SHOTMAP_DIRECTORY = "shotmaps";

    // Seed points distilled from the 2026-04-17 grid calibration session
    // (median hood/RPM per 0.5 m distance bucket). {distance_m, hood_deg, rpm}
    public static final double[][] HUB_DEFAULT_POINTS = {
        { 1.5, 0.0, 2350 },
        { 2.0, 10.5, 2550 },
        { 2.5, 12.0, 2650 },
        { 3.0, 15.0, 2725 },
        { 3.5, 16.0, 2850 },
        { 4.0, 18.0, 3000 },
        { 4.5, 18.0, 3100 },
        { 5.0, 18.0, 3250 },
        { 5.5, 18.0, 3350 },
        { 6.0, 18.0, 3425 },
    };
    public static final double[][] LOB_DEFAULT_POINTS = {
        { 4.0, 24.2, 2820 },
        { 5.0, 24.4, 2960 },
        { 6.0, 24.5, 3340 },
        { 7.0, 24.6, 3530 },
        { 8.0, 24.7, 4220 },
        { 9.0, 24.7, 4420 },
        { 10.0, 24.75, 4520 },
        { 11.0, 24.8, 4700 },
        { 15.0, 24.85, 4700 },
    };
  }

  public static class QuestNavConstants {
    public static final boolean ENABLED = true;

    // Transform from robot center to Quest headset mounting position
    public static final double QUEST_OFFSET_X_METERS = Units.inchesToMeters(-9.213); // Forward/backward from robot
                                                                                     // center
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

  public static class DiagnosticConstants {
    public static final String LOG_DIRECTORY = "/home/lvuser/logs/";
    public static final boolean TURRET_LOGGING_DEFAULT_ENABLED = true;
  }

  public static class Vision {
    // Camera configuration record for multi-camera support
    public record CameraConfig(String name, Transform3d robotToCam) {
    }

    // Left camera
    public static final CameraConfig LEFT_CAMERA = new CameraConfig("LEFT_CAMERA",
        new Transform3d(
            new Translation3d(Units.inchesToMeters(20.988), Units.inchesToMeters(17.903), Units.inchesToMeters(5.120)),
            new Rotation3d(Math.toRadians(40), Math.toRadians(0), Math.toRadians(63))));

    // Right camera
    public static final CameraConfig RIGHT_CAMERA = new CameraConfig("RIGHT_CAMERA",
        new Transform3d(
            new Translation3d(Units.inchesToMeters(20.988), Units.inchesToMeters(-11.903), Units.inchesToMeters(5.120)),
            new Rotation3d(Math.toRadians(-40), Math.toRadians(0), Math.toRadians(-63))));

    // Limelight 4
    public static final CameraConfig LIMELIGHT = new CameraConfig("LIMELIGHT",
        new Transform3d(
            new Translation3d(Units.inchesToMeters(-10.941), Units.inchesToMeters(8.250), Units.inchesToMeters(15.934)),
            new Rotation3d(Math.toRadians(0), Math.toRadians(0), Math.toRadians(180))));

    // All cameras
    public static final List<CameraConfig> CAMERAS = List.of(LEFT_CAMERA, RIGHT_CAMERA);

    // The layout of the AprilTags on the field
    public static final AprilTagFieldLayout kTagLayout = AprilTagFieldLayout.loadField(AprilTagFields.kDefaultField);

    // The standard deviations of our vision estimated poses, which affect
    // correction rate
    public static final Matrix<N3, N1> kSingleTagStdDevs = VecBuilder.fill(4, 4, 8);
    public static final Matrix<N3, N1> kMultiTagStdDevs = VecBuilder.fill(0.5, 0.5, 1);

    // Pose validity timeout - max age in seconds for a pose to be considered valid
    public static final double POSE_VALIDITY_TIMEOUT = 0.5;

  }

}