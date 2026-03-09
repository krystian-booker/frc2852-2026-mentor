package frc.robot;

import frc.robot.Constants.ClimbConstants;
import frc.robot.Constants.OperatorConstants;
import frc.robot.Constants.QuestNavConstants;
import frc.robot.commands.ShootCommand;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.Conveyor;
import frc.robot.subsystems.Flywheel;
import frc.robot.subsystems.Hood;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.IntakeActuator;
import frc.robot.subsystems.Turret;
import frc.robot.subsystems.Climb;
import frc.robot.subsystems.Vision;
import frc.robot.util.Telemetry;
import frc.robot.util.TurretAimingCalculator;

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
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.QuestNavSubsystem;

public class RobotContainer {

  // Subsystems
  private final Conveyor conveyor = new Conveyor();
  private final Flywheel flywheel = new Flywheel();
  private final Hood hood = new Hood();
  private final Intake intake = new Intake();
  private final IntakeActuator intakeActuator = new IntakeActuator();
  private final Turret turret = new Turret();
  private final Climb climb = new Climb();
  // private final LED led = new LED();

  // Controllers
  private final CommandXboxController driverController = new CommandXboxController(
      OperatorConstants.DRIVER_CONTROLLER_PORT);
  private final CommandXboxController operatorController = new CommandXboxController(
      OperatorConstants.OPERATOR_CONTROLLER_PORT);

  // Swerve constants
  private double MaxSpeed = 1.0 * TunerConstants.kSpeedAt12Volts.in(MetersPerSecond);
  private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond);

  private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
      .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1)
      .withDriveRequestType(DriveRequestType.OpenLoopVoltage);

  // Swerve setup
  private final Telemetry logger = new Telemetry(MaxSpeed);
  public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();

  // Vision
  private Vision vision = null;
  private QuestNavSubsystem questNav = null;

  // Turret aiming calculator
  private final TurretAimingCalculator turretAimingCalculator = null;

  // QuestNav seeding state
  private boolean isQuestNavSeeded = false;
  private Pose2d seededPose = null;

  // Auto setup
  private final SendableChooser<Command> autoChooser;

  public RobotContainer() {
    DriverStation.silenceJoystickConnectionWarning(true);

    autoChooser = AutoBuilder.buildAutoChooser("Tests");
    SmartDashboard.putData("Auto Mode", autoChooser);

    // Initialize vision subsystems (after drivetrain)
    vision = new Vision(drivetrain::addVisionMeasurement);
    questNav = new QuestNavSubsystem(drivetrain, vision);

    // Initialize turret aiming calculator with pose supplier from drivetrain
    // turretAimingCalculator = new TurretAimingCalculator(() -> drivetrain.getState().Pose);

    // Set turret default command - auto-aim with operator stick override
    // turret.setDefaultCommand(turret.run(() -> {
    // double stickX = operatorController.getLeftX();
    // double stickY = operatorController.getLeftY();
    // double magnitude = Math.hypot(stickX, stickY);

    // if (magnitude > 0.15) {
    // // Manual field-oriented override
    // double fieldAngleRad = Math.atan2(-stickX, -stickY);
    // double robotHeadingRad = drivetrain.getState().Pose.getRotation().getRadians();
    // double turretAngleDeg = Math.toDegrees(fieldAngleRad - robotHeadingRad) % 360.0;
    // if (turretAngleDeg > 180.0)
    // turretAngleDeg -= 360.0;
    // else if (turretAngleDeg <= -180.0)
    // turretAngleDeg += 360.0;
    // turret.setPosition(turretAngleDeg);
    // } else {
    // // Auto-aim at target
    // var result = turretAimingCalculator.calculate();
    // turret.setPosition(result.turretAngleDegrees());
    // }
    // }).withName("TurretAimWithOverride"));

    // Set intake default command - always running
    intake.setDefaultCommand(intake.run(intake::runIntake));

    // Configure normal bindings (always available)
    configureDriverBindings();
    configureOperatorBindings();
    configureTestBindings();
    questNavInitialization();

    // Warmup PathPlanner to avoid Java pauses
    CommandScheduler.getInstance().schedule(FollowPathCommand.warmupCommand());

    // Telemetry setup
    drivetrain.registerTelemetry(logger::telemeterize);

  }

  private void configureDriverBindings() {
    // Auto-extend intake actuator at the start of autonomous and teleop
    RobotModeTriggers.autonomous().onTrue(intakeActuator.extend());
    RobotModeTriggers.teleop().onTrue(intakeActuator.extend());
    RobotModeTriggers.test().onTrue(intakeActuator.extend());

    // RIGHT TRIGGER - Shoot (held)
    // Spins up flywheel and sets hood from LUT, then feeds when ready
    // Locks wheels in X-brake while shooting unless driver is actively driving
    driverController.rightTrigger(0.5).whileTrue(
        new ShootCommand(flywheel, hood, conveyor, intakeActuator, turret,
            turretAimingCalculator,
            drivetrain,
            () -> drive.withVelocityX(driverController.getLeftY() * MaxSpeed)
                .withVelocityY(driverController.getLeftX() * MaxSpeed)
                .withRotationalRate(-driverController.getRightX()
                    * MaxAngularRate),
            () -> {
              double stickMag = Math.hypot(driverController.getLeftX(),
                  driverController.getLeftY());
              return stickMag > 0.1
                  || Math.abs(driverController.getRightX()) > 0.1;
            })
                .withName("Shoot"));

    // DEFAULT COMMAND - Field-Centric Drive
    // Note that X is defined as forward according to WPILib convention,
    // and Y is defined as to the left according to WPILib convention.
    // Left Stick: Controls translation (forward/backward and left/right)
    // Right Stick X: Controls rotation (counterclockwise is positive)
    // The negatives account for controller axis inversion
    drivetrain.setDefaultCommand(
        drivetrain
            .applyRequest(() -> drive
                .withVelocityX(driverController.getLeftY() * MaxSpeed)
                .withVelocityY(driverController.getLeftX() * MaxSpeed)
                .withRotationalRate(-driverController.getRightX()
                    * MaxAngularRate)));

    // DISABLED MODE - Idle Request
    // Ensures the configured neutral mode (coast) is applied to
    // drive motors while the robot is disabled
    final var idle = new SwerveRequest.Idle();
    RobotModeTriggers.disabled().whileTrue(
        drivetrain.applyRequest(() -> idle).ignoringDisable(true));

    // A BUTTON - Brake Mode
    // Locks all 4 swerve modules into an X-pattern to prevent robot movement
    // driverController.a().whileTrue(drivetrain.applyRequest(() -> brake));

    // D-PAD UP - Robot-Centric Forward
    // Drives straight forward at 0.5 m/s relative to robot's current heading
    // driverController.povUp()
    // .whileTrue(drivetrain.applyRequest(() -> forwardStraight.withVelocityX(0.5).withVelocityY(0)));

    // D-PAD DOWN - Robot-Centric Backward
    // Drives straight backward at 0.5 m/s relative to robot's current heading
    // driverController.povDown()
    // .whileTrue(drivetrain.applyRequest(() -> forwardStraight.withVelocityX(-0.5).withVelocityY(0)));

    // LEFT BUMPER - Reset Field-Centric Heading
    // Resets the gyro's "forward" direction to the robot's current facing direction
    // Use this when the gyro drifts or after manually repositioning the robot
    // driverController.leftBumper().onTrue(drivetrain.runOnce(drivetrain::seedFieldCentric));
  }

  private void configureOperatorBindings() {
    // D-pad controls for climb positions
    operatorController.povUp()
        .onTrue(Commands.runOnce(
            () -> climb.setPosition(ClimbConstants.FULL_EXTENSION_POSITION),
            climb));
    operatorController.povDown()
        .onTrue(Commands.runOnce(() -> climb.setPosition(ClimbConstants.CLIMB_LIFT_POSITION),
            climb));
    operatorController.povLeft()
        .onTrue(Commands.runOnce(() -> climb.setPosition(ClimbConstants.FULLY_DOWN), climb));

    // Climb manual jog controls for zeroing
    RobotModeTriggers.test().and(operatorController.rightBumper()
        .whileTrue(climb.run(() -> climb.manualMove(12.0)).finallyDo(() -> climb.stop())));
    RobotModeTriggers.test().and(operatorController.leftBumper()
        .whileTrue(climb.run(() -> climb.manualMove(-12.0)).finallyDo(() -> climb.stop())));
  }

  /**
   * Configure disabled mode bindings for QuestNav seeding. LEDs start RED and turn GREEN once QuestNav is seeded from a
   * multi-tag vision pose. Seeding only happens while disabled, every 5 seconds. If the robot is moved after seeding,
   * LEDs turn RED and re-seeding is allowed.
   */
  private void questNavInitialization() {
    Command seedingCommand = Commands.sequence(
        Commands.waitSeconds(QuestNavConstants.SEEDING_POLL_INTERVAL_SECONDS),
        Commands.runOnce(() -> {
          // Check if robot has been moved since last seeding using vision pose
          if (isQuestNavSeeded && seededPose != null) {
            var visionPose = vision.getLatestPose2d();
            if (visionPose.isPresent() && vision.getVisibleTagCount() >= 2) {
              double distance = visionPose.get().getTranslation()
                  .getDistance(seededPose.getTranslation());
              if (distance > QuestNavConstants.SEEDING_MOVEMENT_THRESHOLD_METERS) {
                // Robot was moved, reset seeding state
                isQuestNavSeeded = false;
                seededPose = null;
                questNav.clearSeeded();
                vision.setFeedingEnabled(true);
              }
            }
          }

          // Attempt seeding if not seeded and 2+ tags visible
          if (!isQuestNavSeeded && vision.getVisibleTagCount() >= 2) {
            if (questNav.seedPoseFromVision()) {
              isQuestNavSeeded = true;
              seededPose = questNav.getLatestPose();
              vision.setFeedingEnabled(false);
            }
          }

          SmartDashboard.putBoolean("QuestNav/SeededFromVision", isQuestNavSeeded);
        })).repeatedly().ignoringDisable(true);
    RobotModeTriggers.disabled().whileTrue(seedingCommand);

    // Turn off LEDs when auto or teleop starts
    // RobotModeTriggers.autonomous().onTrue(led.setPatternCommand(LED.Pattern.BLACK));
    // RobotModeTriggers.teleop().onTrue(led.setPatternCommand(LED.Pattern.BLACK));
  }

  /**
   * Configure test mode bindings using RobotModeTriggers. These bindings are only active when the robot is in test
   * mode.
   */
  private void configureTestBindings() {

    // Turret Calibration Mode
    // Right bumper toggles calibration mode - reads NetworkTables inputs
    // applies to hood/flywheel in real-time
    // TurretCalibrationCommand calibrationCmd = new TurretCalibrationCommand(
    // hood, flywheel, conveyor, () -> drivetrain.getState().Pose, turretAimingCalculator);

    // Only allow toggling calibration mode while in test mode
    // RobotModeTriggers.test().and(driverController.rightBumper()).toggleOnTrue(calibrationCmd);

    // Swerve
    // RobotModeTriggers.test().and(driverController.a()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kForward));
    // RobotModeTriggers.test().and(driverController.b()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kReverse));
    // RobotModeTriggers.test().and(driverController.x()).whileTrue(drivetrain.sysIdDynamic(Direction.kForward));
    // RobotModeTriggers.test().and(driverController.y()).whileTrue(drivetrain.sysIdDynamic(Direction.kReverse));

    // --- Vision ---
    // A: Toggle vision feeding on | B: Toggle vision feeding off
    // RobotModeTriggers.test().and(driverController.a()).onTrue(Commands.runOnce(() ->
    // vision.setFeedingEnabled(true)));
    // RobotModeTriggers.test().and(driverController.b()).onTrue(Commands.runOnce(() ->
    // vision.setFeedingEnabled(false)));

    // -------------------------------------
    // ---------------DONE------------------
    // -------------------------------------

    // --- Conveyor ---
    // RobotModeTriggers.test().and(driverController.rightBumper()).whileTrue(conveyor.run(conveyor::runFeed));
    // RobotModeTriggers.test().and(driverController.leftBumper()).onTrue(Commands.runOnce(conveyor::stop,
    // conveyor));

    // --- Intake Actuator ---
    // SmartDashboard.putNumber("IntakeActuatorSet Position", 0.0);
    // RobotModeTriggers.test().and(driverController.a()).whileTrue(intakeActuator.extend());
    // RobotModeTriggers.test().and(driverController.b()).whileTrue(intakeActuator.retract());
    // RobotModeTriggers.test().and(driverController.x()).whileTrue(intakeActuator.agitate());

    // --- Intake ---
    // RobotModeTriggers.test().and(driverController.leftBumper()).whileTrue(intake.run(intake::runIntake));
    // RobotModeTriggers.test().and(driverController.x()).onTrue(Commands.runOnce(intake::stop, intake));

    // --- Climb ---
    // RobotModeTriggers.test().and(driverController.a())
    // .whileTrue(climb.run(climb::testDirectionPositive).finallyDo(() -> climb.stop()));
    // RobotModeTriggers.test().and(driverController.b())
    // .whileTrue(climb.run(climb::testDirectionNegative).finallyDo(() -> climb.stop()));
    // RobotModeTriggers.test().and(driverController.x())
    // .onTrue(Commands.runOnce(() -> climb.nudge(1.0), climb));
    // RobotModeTriggers.test().and(driverController.y())
    // .onTrue(Commands.runOnce(() -> climb.nudge(-1.0), climb));
    // RobotModeTriggers.test().and(driverController.leftBumper())
    // .onTrue(Commands.runOnce(climb::stop, climb));

    // --- Turret Manual Test ---
    // RobotModeTriggers.test().and(driverController.a())
    // .whileTrue(turret.run(turret::testDirectionPositive).finallyDo(() -> turret.stop()));
    // RobotModeTriggers.test().and(driverController.b())
    // .whileTrue(turret.run(turret::testDirectionNegative).finallyDo(() -> turret.stop()));
    // RobotModeTriggers.test().and(driverController.x())
    // .onTrue(Commands.runOnce(() -> turret.setPosition(0)));
    // RobotModeTriggers.test().and(driverController.y())
    // .onTrue(Commands.runOnce(() -> turret.nudge(-120)));
    // RobotModeTriggers.test().and(driverController.leftBumper()).onTrue(Commands.runOnce(turret::stop,
    // turret));

    // --- Turret Field Hold ---
    // RobotModeTriggers.test().and(driverController.rightBumper())
    // .whileTrue(turret.fieldHoldCommand(() -> drivetrain.getState().Pose.getRotation().getDegrees()));
    // RobotModeTriggers.test().and(driverController.leftBumper()).onTrue(Commands.runOnce(turret::stop,
    // turret));

    // --- Flywheel Auto-Tune ---
    // RobotModeTriggers.test().and(driverController.back()).toggleOnTrue(new
    // FlywheelAutoTuneCommand(flywheel));

    // --- Flywheel ---
    // RobotModeTriggers.test().and(driverController.a()).whileTrue(flywheel.run(() ->
    // flywheel.setVelocity(2000)));
    // RobotModeTriggers.test().and(driverController.b()).whileTrue(flywheel.run(() ->
    // flywheel.setVelocity(3500)));
    // RobotModeTriggers.test().and(driverController.x()).whileTrue(flywheel.run(() ->
    // flywheel.setVelocity(4500)));
    // RobotModeTriggers.test().and(driverController.y())
    // .onTrue(Commands.runOnce(() -> flywheel.setVelocity(0), flywheel));

    // --- Hood Auto-Tune ---
    // RobotModeTriggers.test().and(driverController.start()).toggleOnTrue(new HoodAutoTuneCommand(hood));

    // --- Hood ---
    // RobotModeTriggers.test().and(driverController.a())
    // .whileTrue(hood.run(hood::testDirectionPositive).finallyDo(() -> hood.setNeutral()));
    // RobotModeTriggers.test().and(driverController.b())
    // .whileTrue(hood.run(hood::testDirectionNegative).finallyDo(() -> hood.setNeutral()));
    // RobotModeTriggers.test().and(driverController.x())
    // .onTrue(hood.runOnce(() -> hood.nudge(5)));
    // RobotModeTriggers.test().and(driverController.y())
    // .onTrue(hood.runOnce(() -> hood.nudge(-5)));
    // RobotModeTriggers.test().and(driverController.leftBumper())
    // .onTrue(Commands.runOnce(hood::setNeutral, hood));

  }

  public Command getAutonomousCommand() {
    return autoChooser.getSelected();
  }
}
