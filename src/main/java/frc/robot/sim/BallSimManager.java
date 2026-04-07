package frc.robot.sim;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.Constants.BallSimConstants;
import frc.robot.Constants.HoodConstants;
import frc.robot.Constants.TurretAimingConstants;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.flywheel.Flywheel;
import frc.robot.subsystems.hood.Hood;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.turret.Turret;
import frc.robot.util.firecontrol.FuelPhysicsSim;
import org.littletonrobotics.junction.Logger;

/**
 * Simulation-only coordinator that wires FuelPhysicsSim to robot subsystems. Fires balls whenever
 * the flywheel is at speed (unlimited ammo for tuning). Computes 3D launch parameters from
 * flywheel/hood/turret state.
 */
public class BallSimManager {

  private final Drive drive;
  private final Flywheel flywheel;
  private final Hood hood;
  private final Turret turret;
  private final Intake intake;

  private FuelPhysicsSim ballSim;
  private double lastLaunchTime = 0;

  public BallSimManager(Drive drive, Flywheel flywheel, Hood hood, Turret turret, Intake intake) {
    this.drive = drive;
    this.flywheel = flywheel;
    this.hood = hood;
    this.turret = turret;
    this.intake = intake;
  }

  /** Called from simulationInit(). Creates and configures the ball physics sim. */
  public void init() {
    ballSim = new FuelPhysicsSim("Sim/Fuel");

    // Configure robot dimensions and suppliers
    // configureRobot expects field-relative ChassisSpeeds
    ballSim.configureRobot(
        BallSimConstants.ROBOT_WIDTH_M,
        BallSimConstants.ROBOT_LENGTH_M,
        BallSimConstants.BUMPER_HEIGHT_M,
        drive::getPose,
        () -> {
          ChassisSpeeds robotRelative = drive.getChassisSpeeds();
          return ChassisSpeeds.fromRobotRelativeSpeeds(
              robotRelative, drive.getPose().getRotation());
        });

    // Intake zone at front of robot (robot-relative coordinates)
    ballSim.addIntakeZone(
        0.25,
        0.45, // xMin, xMax: front edge
        -0.20,
        0.20, // yMin, yMax: centered
        () -> intake.getVelocityRPS() > 1.0);

    ballSim.enable();
  }

  /** Called from simulationPeriodic(). Detects launches, steps physics, logs telemetry. */
  public void tick() {
    if (ballSim == null) return;

    detectAndLaunch();
    ballSim.tick();
    logTelemetry();
  }

  private void detectAndLaunch() {
    // Unlimited ammo: fire a ball whenever the flywheel is at speed (shoot button held)
    boolean flywheelAtSpeed = flywheel.atSetpoint() && flywheel.getCurrentVelocityRPM() > 500;
    double now = Timer.getFPGATimestamp();

    if (flywheelAtSpeed && (now - lastLaunchTime) > BallSimConstants.LAUNCH_COOLDOWN_SECONDS) {
      fireBall();
      lastLaunchTime = now;
    }
  }

  private void fireBall() {
    Pose2d robotPose = drive.getPose();
    double robotHeadingRad = robotPose.getRotation().getRadians();

    // Exit speed from flywheel RPM
    double flywheelRPM = flywheel.getCurrentVelocityRPM();
    double exitSpeed =
        BallSimConstants.SLIP_FACTOR
            * flywheelRPM
            * Math.PI
            * BallSimConstants.WHEEL_DIAMETER_M
            / 60.0;

    // Launch elevation from hood mechanism position
    double mechanismDeg = hood.getCurrentPositionDegrees();
    double elevationDeg = HoodConstants.mechanismToActualAngle(mechanismDeg);
    double elevationRad = Math.toRadians(elevationDeg);

    // Launch azimuth from turret angle + robot heading
    double turretAngleRad = Math.toRadians(turret.getPositionDegrees());
    double fieldAzimuthRad = robotHeadingRad + turretAngleRad;

    // 3D velocity in field frame
    double vHorizontal = exitSpeed * Math.cos(elevationRad);
    double vVertical = exitSpeed * Math.sin(elevationRad);
    double vx = vHorizontal * Math.cos(fieldAzimuthRad);
    double vy = vHorizontal * Math.sin(fieldAzimuthRad);

    // Add robot velocity (field-relative)
    ChassisSpeeds robotSpeeds = drive.getChassisSpeeds();
    ChassisSpeeds fieldSpeeds =
        ChassisSpeeds.fromRobotRelativeSpeeds(robotSpeeds, robotPose.getRotation());

    Translation3d launchVel =
        new Translation3d(
            vx + fieldSpeeds.vxMetersPerSecond, vy + fieldSpeeds.vyMetersPerSecond, vVertical);

    // Launch position: robot center + turret offset rotated to field frame
    double turretX = TurretAimingConstants.TURRET_OFFSET_X_METERS;
    double turretY = TurretAimingConstants.TURRET_OFFSET_Y_METERS;
    double cosH = Math.cos(robotHeadingRad);
    double sinH = Math.sin(robotHeadingRad);
    double fieldTurretX = robotPose.getX() + turretX * cosH - turretY * sinH;
    double fieldTurretY = robotPose.getY() + turretX * sinH + turretY * cosH;

    Translation3d launchPos =
        new Translation3d(fieldTurretX, fieldTurretY, BallSimConstants.EXIT_HEIGHT_M);

    // Spin (backspin)
    double spinRPM = flywheelRPM * BallSimConstants.BACKSPIN_FRACTION;

    ballSim.launchBall(launchPos, launchVel, spinRPM);
  }

  private void logTelemetry() {
    Logger.recordOutput("BallSim/BlueScore", ballSim.getBlueScore());
    Logger.recordOutput("BallSim/RedScore", ballSim.getRedScore());
    Logger.recordOutput("BallSim/BallCount", ballSim.getBallCount());
    Logger.recordOutput("BallSim/InFlight", ballSim.getBallsInFlight());
    Logger.recordOutput("BallSim/TotalLaunched", ballSim.getTotalLaunched());
    Logger.recordOutput("BallSim/TotalScored", ballSim.getTotalScored());
    Logger.recordOutput("BallSim/LastLaunchSpeed", ballSim.getLastLaunchSpeed());
  }
}
