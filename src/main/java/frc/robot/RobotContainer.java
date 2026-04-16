package frc.robot;

import frc.robot.Constants.IntakeActuatorConstants;
import frc.robot.Constants.OperatorConstants;
import frc.robot.Constants.QuestNavConstants;
import frc.robot.Constants.TurretConstants;
import frc.robot.Constants.CalibrationConstants;
import frc.robot.commands.DumbShootCommand;
import frc.robot.commands.ShootCommand;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.Flywheel;
import frc.robot.subsystems.Hood;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.IntakeActuator;
import frc.robot.subsystems.Turret;
import frc.robot.subsystems.Vision;
import frc.robot.util.DiagnosticLogger;
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
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.FollowPathCommand;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.networktables.DoubleSubscriber;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.QuestNavSubsystem;

public class RobotContainer {

  // Subsystems
  private final Indexer indexer = new Indexer();
  private final Flywheel flywheel = new Flywheel();
  private final Hood hood = new Hood();
  private final Intake intake = new Intake();
  private final IntakeActuator intakeActuator = new IntakeActuator();
  private final Turret turret = new Turret();

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
  private TurretAimingCalculator shooterCalculator = null;

  // Shared shot logger - one CSV per enabled session
  private static final String[] SHOT_LOG_COLUMNS = {
      "MatchTime", "RobotX", "RobotY", "TargetRPM", "ActualRPM",
      "TargetHoodAngle", "ActualHoodAngle", "Feeding"
  };
  private final DiagnosticLogger shotLogger = new DiagnosticLogger("Shoot", SHOT_LOG_COLUMNS);

  // Physical reseed button on the robot
  private final DigitalInput reseedButtonInput = new DigitalInput(QuestNavConstants.RESEED_BUTTON_DIO_PORT);
  private final Trigger reseedButton = new Trigger(() -> !reseedButtonInput.get());

  // Auto setup
  private final SendableChooser<Command> autoChooser;

  // Calibration webapp subscribers for test-mode default control
  private final NetworkTable calibrationTable = NetworkTableInstance.getDefault().getTable("TurretCalibration");
  private final DoubleSubscriber calibrationHoodAngleSub = calibrationTable.getDoubleTopic("Input/HoodAngle")
      .subscribe(CalibrationConstants.DEFAULT_HOOD_ANGLE);
  private final DoubleSubscriber calibrationFlywheelRPMSub = calibrationTable.getDoubleTopic("Input/FlywheelRPM")
      .subscribe(CalibrationConstants.DEFAULT_FLYWHEEL_RPM);

  public RobotContainer() {
    DriverStation.silenceJoystickConnectionWarning(true);

    // Initialize vision subsystems
    vision = new Vision(drivetrain::addVisionMeasurement);
    if (QuestNavConstants.ENABLED) {
      questNav = new QuestNavSubsystem(drivetrain, vision);
    }
    shooterCalculator = new TurretAimingCalculator(
        () -> drivetrain.getState().Pose,
        () -> drivetrain.getState().Speeds);

    // Register named commands before building auto chooser
    NamedCommands.registerCommand("shoot",
        new ShootCommand(flywheel, hood, indexer, intake, turret, shooterCalculator, shotLogger));
    NamedCommands.registerCommand("extendIntake", intakeActuator.extend());
    NamedCommands.registerCommand("agitateIntake", intakeActuator.agitate());
    NamedCommands.registerCommand("runIntake", intake.runCommand());

    autoChooser = AutoBuilder.buildAutoChooser();
    autoChooser.addOption("Drive Back & Shoot", buildDriveBackAndShootAuto());
    SmartDashboard.putData("Auto Mode", autoChooser);
    SmartDashboard.putNumber("IntakeActuator/StepTestDutyCycle", IntakeActuatorConstants.STEP_TEST_DUTY_CYCLE);

    // Set turret default command - auto-aim with operator stick override
    turret.setDefaultCommand(turret.run(() -> {
      double stickX = operatorController.getLeftX();
      double stickY = operatorController.getLeftY();
      double magnitude = Math.hypot(stickX, stickY);

      double turretSetpoint;
      if (magnitude > 0.15) {
        // Manual field-oriented override
        double fieldAngleRad = Math.atan2(-stickX, -stickY);
        double robotHeadingRad = drivetrain.getState().Pose.getRotation().getRadians();
        double turretAngleDeg = Math.toDegrees(fieldAngleRad - robotHeadingRad) %
            360.0;
        if (turretAngleDeg > 180.0)
          turretAngleDeg -= 360.0;
        else if (turretAngleDeg <= -180.0)
          turretAngleDeg += 360.0;
        if (turretAngleDeg > TurretConstants.MAX_POSITION_DEGREES)
          turretAngleDeg -= 360.0;
        turretSetpoint = turretAngleDeg;
        turret.setPosition(turretSetpoint);
      } else {
        // Auto-aim at target with SOTM
        var result = shooterCalculator.calculate();
        turretSetpoint = result.turretAngleDegrees();

        // Feedforward: turret counter-rotates against chassis yaw to maintain field
        // heading
        double chassisOmega = shooterCalculator.getChassisOmegaDegreesPerSecond();
        turret.setPosition(turretSetpoint, -chassisOmega);
      }
    }).withName("TurretAimWithOverride"));

    hood.setDefaultCommand(hood.run(() -> {
      if (DriverStation.isTest()) {
        hood.setPosition(calibrationHoodAngleSub.get());
      }
    }).withName("HoodCalibrationWebappDefault"));

    flywheel.setDefaultCommand(flywheel.run(() -> {
      if (DriverStation.isTest()) {
        flywheel.setVelocity(calibrationFlywheelRPMSub.get());
      }
    }).withName("FlywheelCalibrationWebappDefault"));

    // Configure normal bindings (always available)
    configureDriverBindings();
    configureOperatorBindings();
    configureTestBindings();
    if (QuestNavConstants.ENABLED) {
      questNavInitialization();
    }

    // Warmup PathPlanner to avoid Java pauses
    CommandScheduler.getInstance().schedule(FollowPathCommand.warmupCommand());

    // Telemetry setup
    drivetrain.registerTelemetry(logger::telemeterize);
  }

  private void configureDriverBindings() {
    // Open shot logger when robot enters any enabled mode, close on disable
    RobotModeTriggers.teleop().onTrue(Commands.runOnce(() -> shotLogger.open()));
    RobotModeTriggers.autonomous().onTrue(Commands.runOnce(() -> shotLogger.open()));
    RobotModeTriggers.disabled().onTrue(Commands.runOnce(() -> shotLogger.close()));

    // Auto-extend intake actuator at the start of autonomous and teleop
    RobotModeTriggers.teleop().onTrue(intakeActuator.extend());

    // LEFT TRIGGER - Intake (held)
    // Runs the intake roller to acquire game pieces
    driverController.leftTrigger(0.5).whileTrue(intake.run(intake::runIntake).finallyDo(intake::stop));

    // RIGHT TRIGGER - Shoot (held)
    RobotModeTriggers.teleop().and(driverController.rightTrigger(0.5)).whileTrue(
        new ShootCommand(flywheel, hood, indexer, intake, turret,
            shooterCalculator,
            intakeActuator,
            shotLogger)
            .withName("Shoot"));

    // LEFT BUMPER - Retract intake actuator (held)
    operatorController.a().onTrue(intakeActuator.retract());
    operatorController.b().onTrue(intakeActuator.extend());
    operatorController.leftTrigger(0.5).whileTrue(
        Commands.parallel(indexer.runReverseCommand(), intake.runOutCommand()));

    // RIGHT BUMPER - Shoot without retracting intake actuator (held)
    RobotModeTriggers.teleop().and(driverController.rightBumper()).whileTrue(
        new ShootCommand(flywheel, hood, indexer, intake, turret,
            shooterCalculator, shotLogger)
            .withName("ShootNoRetract"));

    // DEFAULT COMMAND - Field-Centric Drive
    drivetrain.setDefaultCommand(
        drivetrain.applyRequest(this::getDriveRequest));

    // DISABLED MODE - Idle Request
    final var idle = new SwerveRequest.Idle();
    RobotModeTriggers.disabled().whileTrue(
        drivetrain.applyRequest(() -> idle).ignoringDisable(true));
  }

  private void configureOperatorBindings() {

  }

  /**
   * Configure QuestNav seeding. Continuously reseeds QuestNav from vision
   * whenever 2+ AprilTags are visible, in all robot modes.
   */
  private void questNavInitialization() {
    // Continuous reseeding: call seedPoseFromVision() every cycle in all modes.
    // It internally checks for 2+ tags and a recent valid vision pose;
    // when conditions aren't met it safely no-ops.
    questNav.setDefaultCommand(
        Commands.run(() -> questNav.seedPoseFromVision(), questNav)
            .ignoringDisable(true)
            .withName("QuestNavContinuousReseed"));

    // Reseed button: clears seeded state so QuestNav stops feeding drivetrain
    // until next successful 2+ tag seed.
    reseedButton.onTrue(
        Commands.runOnce(() -> questNav.clearSeeded())
            .ignoringDisable(true));
  }

  /**
   * Configure test mode bindings using RobotModeTriggers. These bindings are only
   * active when the robot is in test
   * mode.
   */
  private void configureTestBindings() {
    // Calibration session runs for all of test mode. Hood/flywheel continuously
    // follow webapp setpoints via their default commands, and the right trigger
    // only feeds game pieces.
    // TurretCalibrationCommand calibrationCmd = new TurretCalibrationCommand(
    // hood, flywheel, indexer,
    // () -> drivetrain.getState().Pose, shooterCalculator,
    // drivetrain, this::getDriveRequest, this::isDriverActive,
    // () -> driverController.getRightTriggerAxis() > 0.5);
    // RobotModeTriggers.test().whileTrue(calibrationCmd);
    // RobotModeTriggers.test().and(driverController.rightTrigger(0.5)).whileTrue(
    // Commands.parallel(
    // indexer.feedCommand(),
    // intake.runCommand())
    // .withName("TurretCalibrationFeed"));
    // RobotModeTriggers.test().onFalse(Commands.runOnce(() -> {
    // hood.setNeutral();
    // flywheel.stop();
    // indexer.stop();
    // intake.stop();
    // }, hood, flywheel, indexer, intake));

    // --- Indexer ---
    // RobotModeTriggers.test().and(driverController.povUp()).whileTrue(indexer.feedCommand());
    // RobotModeTriggers.test().and(driverController.povDown()).onTrue(Commands.runOnce(indexer::stop,
    // indexer));

    // // --- Intake Actuator ---
    // RobotModeTriggers.test().and(driverController.a()).onTrue(intakeActuator.extend());
    // RobotModeTriggers.test().and(driverController.b()).onTrue(intakeActuator.retract());
    // //
    // RobotModeTriggers.test().and(driverController.x()).whileTrue(intakeActuator.agitate());
    // RobotModeTriggers.test().and(driverController.povDown()).whileTrue(
    // intakeActuator.run(intakeActuator::driveRetractOpenLoop).finallyDo(intakeActuator::stop));
    // RobotModeTriggers.test().and(driverController.povLeft()).onTrue(
    // Commands.runOnce(intakeActuator::resetEncoder, intakeActuator));

    // // --- Intake ---
    // RobotModeTriggers.test().and(driverController.leftBumper()).whileTrue(intake.run(intake::runIntake));
    // RobotModeTriggers.test().and(driverController.y()).onTrue(Commands.runOnce(intake::stop,
    // intake));

    // --- Turret Manual Test ---
    // driverController.a()
    // .whileTrue(turret.run(turret::testDirectionPositive).finallyDo(() ->
    // turret.stop()));
    // driverController.b()
    // .whileTrue(turret.run(turret::testDirectionNegative).finallyDo(() ->
    // turret.stop()));
    RobotModeTriggers.test().and(driverController.x())
        .onTrue(Commands.runOnce(() -> turret.setPosition(0)));
    RobotModeTriggers.test().and(driverController.y())
        .onTrue(Commands.runOnce(() -> turret.nudge(-180)));
    RobotModeTriggers.test().and(driverController.leftBumper()).onTrue(Commands.runOnce(turret::stop,
        turret));

    // --- Turret Field Hold ---
    // RobotModeTriggers.test().and(driverController.rightBumper())
    // .whileTrue(turret.fieldHoldCommand(() ->
    // drivetrain.getState().Pose.getRotation().getDegrees()));
    // RobotModeTriggers.test().and(driverController.leftBumper()).onTrue(Commands.runOnce(turret::stop,
    // turret));

    // --- Turret System Identification (for MATLAB) ---
    // Operator D-pad Up: runs all routines (Quasistatic → Steps → Coastdown)
    // RobotModeTriggers.test().and(operatorController.povUp())
    // .toggleOnTrue(new TurretSysIdCommand(turret,
    // TurretSysIdCommand.Routine.ALL));

    // --- Flywheel System Identification (for MATLAB) ---
    // RobotModeTriggers.test().and(driverController.back())
    // .toggleOnTrue(new FlywheelSysIdCommand(flywheel));

    // --- Flywheel ---
    // RobotModeTriggers.test().and(driverController.a())
    // .whileTrue(new FlywheelTestCommand(flywheel, false, 4000.0));
    // RobotModeTriggers.test().and(driverController.b())
    // .whileTrue(new FlywheelTestCommand(flywheel, true, 4000.0));
    // RobotModeTriggers.test().and(driverController.x()).whileTrue(flywheel.run(()
    // -> flywheel.setVelocity(6000)));
    // RobotModeTriggers.test().and(driverController.y())
    // .onTrue(Commands.runOnce(() -> flywheel.setVelocity(0), flywheel));

    // --- Hood Test Sequence ---
    // Operator start button: zeros hood then runs through full range for diagnostic
    // capture
    // RobotModeTriggers.test().and(driverController.start())
    // .onTrue(HoodTestSequenceCommand.create(hood));

    // --- Hood ---
    // RobotModeTriggers.test().and(driverController.a())
    // .whileTrue(hood.run(hood::testDirectionPositive).finallyDo(() ->
    // hood.setNeutral()));
    // RobotModeTriggers.test().and(driverController.b())
    // .whileTrue(hood.run(hood::testDirectionNegative).finallyDo(() ->
    // hood.setNeutral()));
    // RobotModeTriggers.test().and(driverController.x())
    // .onTrue(hood.runOnce(() -> hood.nudge(5)));
    // RobotModeTriggers.test().and(driverController.y())
    // .onTrue(hood.runOnce(() -> hood.nudge(-5)));
    // RobotModeTriggers.test().and(driverController.leftBumper())
    // .onTrue(Commands.runOnce(hood::setNeutral, hood));

    // Hood manual re-home (start button in test mode)
    // RobotModeTriggers.test().and(driverController.start())
    // .onTrue(hood.zeroHoodCommand());

  }

  /** Returns true if the driver is actively providing joystick input. */
  private boolean isDriverActive() {
    double stickMag = Math.hypot(driverController.getLeftX(), driverController.getLeftY());
    return stickMag > 0.1 || Math.abs(driverController.getRightX()) > 0.1;
  }

  /** Builds a field-centric drive request from driver joystick input. */
  private SwerveRequest getDriveRequest() {
    return drive
        .withVelocityX(-driverController.getLeftY() * MaxSpeed)
        .withVelocityY(-driverController.getLeftX() * MaxSpeed)
        .withRotationalRate(-driverController.getRightX() * MaxAngularRate);
  }

  private Command buildDriveBackAndShootAuto() {
    SwerveRequest.RobotCentric driveBack = new SwerveRequest.RobotCentric()
        .withVelocityX(-1.0) // backwards at 1 m/s
        .withVelocityY(0)
        .withRotationalRate(0);
    SwerveRequest.Idle stop = new SwerveRequest.Idle();

    return Commands.sequence(
        Commands.deadline(
            drivetrain.applyRequest(() -> driveBack)
                .withTimeout(0.6),
            intakeActuator.extend()),
        drivetrain.applyRequest(() -> stop).withTimeout(0.02),
        Commands.parallel(new ShootCommand(flywheel, hood, indexer, intake, turret, shooterCalculator, shotLogger),
            intakeActuator.agitate()));
  }

  public Command getAutonomousCommand() {
    return autoChooser.getSelected();
  }
}
