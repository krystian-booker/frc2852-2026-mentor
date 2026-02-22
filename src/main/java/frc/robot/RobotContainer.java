package frc.robot;

import frc.robot.Constants.OperatorConstants;
import frc.robot.Constants.QuestNavConstants;
import frc.robot.commands.TurretCalibrationCommand;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.Flywheel;
import frc.robot.subsystems.Hood;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.IntakeActuator;
import frc.robot.subsystems.Turret;
import frc.robot.subsystems.LED;
import frc.robot.subsystems.Vision;
import frc.robot.util.Telemetry;
import frc.robot.util.TurretAimingCalculator;

import com.ctre.phoenix6.SignalLogger;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.FollowPathCommand;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.QuestNavSubsystem;

public class RobotContainer {

  // Subsystems
  private final Flywheel flywheel = new Flywheel();
  private final Hood hood = new Hood();
  private final Intake intake = new Intake();
  private final IntakeActuator intakeActuator = new IntakeActuator();
  private final Turret turret = new Turret();
  private final LED led = new LED();

  // Controllers
  private final CommandXboxController driverController = new CommandXboxController(
      OperatorConstants.DRIVER_CONTROLLER_PORT);

  // Swerve constants
  private double MaxSpeed = 1.0 * TunerConstants.kSpeedAt12Volts.in(MetersPerSecond);
  private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond);

  private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
      .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1)
      .withDriveRequestType(DriveRequestType.OpenLoopVoltage);
  private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
  private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();
  private final SwerveRequest.RobotCentric forwardStraight = new SwerveRequest.RobotCentric()
      .withDriveRequestType(DriveRequestType.OpenLoopVoltage);

  // Swerve setup
  private final Telemetry logger = new Telemetry(MaxSpeed);
  public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();

  // Vision
  private final Vision vision;
  private final QuestNavSubsystem questNav;

  // Turret aiming calculator
  private final TurretAimingCalculator turretAimingCalculator;

  // QuestNav seeding state
  private boolean isQuestNavSeeded = false;
  private Pose2d seededPose = null;

  // Manual reseed mode state
  private boolean reseedModeActive = false;

  // Auto setup
  private final SendableChooser<Command> autoChooser;

  public RobotContainer() {
    // Initialize vision subsystems (after drivetrain)
    vision = new Vision(drivetrain::addVisionMeasurement);
    questNav = new QuestNavSubsystem(drivetrain, vision);

    // Initialize turret aiming calculator with pose supplier from drivetrain
    turretAimingCalculator = new TurretAimingCalculator(() -> drivetrain.getState().Pose);

    // Set turret default command - continuously track the target
    turret.setDefaultCommand(turret.aimAtTargetCommand(turretAimingCalculator));

    // Warmup PathPlanner to avoid Java pauses
    CommandScheduler.getInstance().schedule(FollowPathCommand.warmupCommand());

    autoChooser = AutoBuilder.buildAutoChooser();
    SmartDashboard.putData("Auto Mode", autoChooser);

    // Configure normal bindings (always available)
    configureBindings();
    configureSwerveBindings();

    // Configure test mode bindings (activated when entering test mode)
    configureTestBindings();

    // Configure disabled mode QuestNav seeding
    configureDisabledBindings();

    // Telemetry setup
    drivetrain.registerTelemetry(logger::telemeterize);

  }

  private void configureBindings() {
    // Auto-extend intake actuator at the start of autonomous and teleop
    RobotModeTriggers.autonomous().onTrue(intakeActuator.extend());
    RobotModeTriggers.teleop().onTrue(intakeActuator.extend());
  }

  private void configureSwerveBindings() {
    // DEFAULT COMMAND - Field-Centric Drive
    // Note that X is defined as forward according to WPILib convention,
    // and Y is defined as to the left according to WPILib convention.
    // Left Stick: Controls translation (forward/backward and left/right)
    // Right Stick X: Controls rotation (counterclockwise is positive)
    // The negatives account for controller axis inversion
    drivetrain.setDefaultCommand(
        drivetrain
            .applyRequest(() -> drive.withVelocityX(-driverController.getLeftY() * MaxSpeed)
                .withVelocityY(-driverController.getLeftX() * MaxSpeed)
                .withRotationalRate(-driverController.getRightX() * MaxAngularRate)));

    // DISABLED MODE - Idle Request
    // Ensures the configured neutral mode (coast) is applied to
    // drive motors while the robot is disabled
    final var idle = new SwerveRequest.Idle();
    RobotModeTriggers.disabled().whileTrue(
        drivetrain.applyRequest(() -> idle).ignoringDisable(true));

    // A BUTTON - Brake Mode
    // Locks all 4 swerve modules into an X-pattern to prevent robot movement
    driverController.a().whileTrue(drivetrain.applyRequest(() -> brake));

    // B BUTTON - Point Wheels
    // Points all wheels in the direction of the left stick without driving
    // Useful for aligning wheels before moving in a specific direction
    driverController.b().whileTrue(drivetrain
        .applyRequest(() -> point
            .withModuleDirection(new Rotation2d(-driverController.getLeftY(), -driverController.getLeftX()))));

    // D-PAD UP - Robot-Centric Forward
    // Drives straight forward at 0.5 m/s relative to robot's current heading
    driverController.povUp()
        .whileTrue(drivetrain.applyRequest(() -> forwardStraight.withVelocityX(0.5).withVelocityY(0)));

    // D-PAD DOWN - Robot-Centric Backward
    // Drives straight backward at 0.5 m/s relative to robot's current heading
    driverController.povDown()
        .whileTrue(drivetrain.applyRequest(() -> forwardStraight.withVelocityX(-0.5).withVelocityY(0)));

    // LEFT BUMPER - Reset Field-Centric Heading
    // Resets the gyro's "forward" direction to the robot's current facing direction
    // Use this when the gyro drifts or after manually repositioning the robot
    driverController.leftBumper().onTrue(drivetrain.runOnce(drivetrain::seedFieldCentric));

    // BACK BUTTON - Enter Reseed Mode
    // Enters manual reseed mode and sets LEDs to RED
    driverController.back().onTrue(Commands.runOnce(() -> {
      reseedModeActive = true;
      led.setPattern(LED.Pattern.RED);
    }));

    // START BUTTON - Execute Reseed (if conditions met)
    // Only seeds if reseed mode is active AND 2+ tags are visible
    driverController.start().onTrue(Commands.either(
        // If reseed mode active and 2+ tags visible: seed and double-flash
        Commands.sequence(
            Commands.runOnce(() -> {
              questNav.seedPoseFromVision();
              reseedModeActive = false;
            }),
            led.doubleFlashAndOffCommand(LED.Pattern.GREEN)),
        // Otherwise do nothing
        Commands.none(),
        () -> reseedModeActive && vision.getVisibleTagCount() >= 2));

    // Tag count monitoring for reseed mode
    // When reseed mode is active and 2+ tags become visible, set LEDs GREEN
    new Trigger(() -> reseedModeActive && vision.getVisibleTagCount() >= 2)
        .onTrue(led.setPatternCommand(LED.Pattern.GREEN));

    // When reseed mode is active and tags drop below 2, set LEDs back to RED
    new Trigger(() -> reseedModeActive && vision.getVisibleTagCount() < 2)
        .onTrue(led.setPatternCommand(LED.Pattern.RED));
  }

  /**
   * Configure disabled mode bindings for QuestNav seeding. LEDs start RED and turn GREEN once QuestNav is seeded from a
   * multi-tag vision pose. Seeding only happens while disabled, every 5 seconds. If the robot is moved after seeding,
   * LEDs turn RED and re-seeding is allowed.
   */
  private void configureDisabledBindings() {
    Command seedingCommand = Commands.sequence(
        Commands.waitSeconds(5.0),
        Commands.runOnce(() -> {
          // Check if robot has been moved since last seeding
          if (isQuestNavSeeded && seededPose != null) {
            Pose2d currentPose = questNav.getLatestPose();
            double distance = currentPose.getTranslation().getDistance(seededPose.getTranslation());
            if (distance > QuestNavConstants.SEEDING_MOVEMENT_THRESHOLD_METERS) {
              // Robot was moved, reset seeding state
              isQuestNavSeeded = false;
              seededPose = null;
              led.setPattern(LED.Pattern.RED);
              vision.setFeedingEnabled(true);
            }
          }

          // Attempt seeding if not seeded and 2+ tags visible
          if (!isQuestNavSeeded && vision.getVisibleTagCount() >= 2) {
            if (questNav.seedPoseFromVision()) {
              isQuestNavSeeded = true;
              seededPose = questNav.getLatestPose();
              led.setPattern(LED.Pattern.GREEN);
              vision.setFeedingEnabled(false);
            }
          }
        })).repeatedly().ignoringDisable(true);

    RobotModeTriggers.disabled().whileTrue(seedingCommand);

    // Turn off LEDs when auto or teleop starts
    RobotModeTriggers.autonomous().onTrue(led.setPatternCommand(LED.Pattern.BLACK));
    RobotModeTriggers.teleop().onTrue(led.setPatternCommand(LED.Pattern.BLACK));
  }

  /**
   * Configure test mode bindings using RobotModeTriggers. These bindings are only active when the robot is in test
   * mode.
   *
   * Button mapping: A - Quasistatic Forward B - Quasistatic Reverse X - Dynamic Forward Y - Dynamic Reverse Right
   * Bumper - Toggle Turret Calibration Mode
   */
  private void configureTestBindings() {
    // Start CTRE SignalLogger when entering test mode
    RobotModeTriggers.test().onTrue(Commands.runOnce(() -> SignalLogger.start()));

    // Turret Calibration Mode
    // Right bumper toggles calibration mode - reads NetworkTables inputs
    // applies to hood/flywheel in real-time
    TurretCalibrationCommand calibrationCmd = new TurretCalibrationCommand(
        hood, flywheel, () -> drivetrain.getState().Pose, turretAimingCalculator);

    // Only allow toggling calibration mode while in test mode
    RobotModeTriggers.test().and(driverController.rightBumper()).toggleOnTrue(calibrationCmd);

    // Swerve
    // RobotModeTriggers.test().and(driverController.a()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kForward));
    // RobotModeTriggers.test().and(driverController.b()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kReverse));
    // RobotModeTriggers.test().and(driverController.x()).whileTrue(drivetrain.sysIdDynamic(Direction.kForward));
    // RobotModeTriggers.test().and(driverController.y()).whileTrue(drivetrain.sysIdDynamic(Direction.kReverse));

    // Flywheel
    // RobotModeTriggers.test().and(driverController.a()).whileTrue(flywheel.sysIdQuasistatic(SysIdRoutine.Direction.kForward));
    // RobotModeTriggers.test().and(driverController.b()).whileTrue(flywheel.sysIdQuasistatic(SysIdRoutine.Direction.kReverse));
    // RobotModeTriggers.test().and(driverController.x()).whileTrue(flywheel.sysIdDynamic(SysIdRoutine.Direction.kForward));
    // RobotModeTriggers.test().and(driverController.y()).whileTrue(flywheel.sysIdDynamic(SysIdRoutine.Direction.kReverse));

    // Hood
    // RobotModeTriggers.test().and(driverController.a()).whileTrue(hood.sysIdQuasistatic(SysIdRoutine.Direction.kForward));
    // RobotModeTriggers.test().and(driverController.b()).whileTrue(hood.sysIdQuasistatic(SysIdRoutine.Direction.kReverse));
    // RobotModeTriggers.test().and(driverController.x()).whileTrue(hood.sysIdDynamic(SysIdRoutine.Direction.kForward));
    // RobotModeTriggers.test().and(driverController.y()).whileTrue(hood.sysIdDynamic(SysIdRoutine.Direction.kReverse));

    // Turret
    // RobotModeTriggers.test().and(driverController.a()).whileTrue(turret.sysIdQuasistatic(SysIdRoutine.Direction.kForward));
    // RobotModeTriggers.test().and(driverController.b()).whileTrue(turret.sysIdQuasistatic(SysIdRoutine.Direction.kReverse));
    // RobotModeTriggers.test().and(driverController.x()).whileTrue(turret.sysIdDynamic(SysIdRoutine.Direction.kForward));
    // RobotModeTriggers.test().and(driverController.y()).whileTrue(turret.sysIdDynamic(SysIdRoutine.Direction.kReverse));

    // Intake
    // RobotModeTriggers.test().and(driverController.a()).whileTrue(intake.sysIdQuasistatic(SysIdRoutine.Direction.kForward));
    // RobotModeTriggers.test().and(driverController.b()).whileTrue(intake.sysIdQuasistatic(SysIdRoutine.Direction.kReverse));
    // RobotModeTriggers.test().and(driverController.x()).whileTrue(intake.sysIdDynamic(SysIdRoutine.Direction.kForward));
    // RobotModeTriggers.test().and(driverController.y()).whileTrue(intake.sysIdDynamic(SysIdRoutine.Direction.kReverse));

    // Intake Actuator
    // RobotModeTriggers.test().and(driverController.a()).whileTrue(intakeActuator.sysIdQuasistatic(SysIdRoutine.Direction.kForward));
    // RobotModeTriggers.test().and(driverController.b()).whileTrue(intakeActuator.sysIdQuasistatic(SysIdRoutine.Direction.kReverse));
    // RobotModeTriggers.test().and(driverController.x()).whileTrue(intakeActuator.sysIdDynamic(SysIdRoutine.Direction.kForward));
    // RobotModeTriggers.test().and(driverController.y()).whileTrue(intakeActuator.sysIdDynamic(SysIdRoutine.Direction.kReverse));
  }

  public Command getAutonomousCommand() {
    return autoChooser.getSelected();
  }
}
