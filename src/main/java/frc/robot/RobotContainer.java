package frc.robot;

import frc.robot.Constants.OperatorConstants;
import frc.robot.Constants.QuestNavConstants;
import frc.robot.Constants.TurretConstants;
import frc.robot.commands.ShootCommand;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.Conveyor;
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
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import edu.wpi.first.wpilibj2.command.button.Trigger;
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

  // Diagnostic logging
  private final DiagnosticLogger turretDiagLogger = new DiagnosticLogger("turret_diag", new String[] {
      "timestamp", "pose_x", "pose_y", "pose_heading",
      "vel_vx", "vel_vy", "vel_omega",
      "target_x", "target_y", "distance",
      "raw_turret_angle", "filtered_turret_angle",
      "turret_setpoint", "turret_actual", "turret_error",
      "motor_voltage", "stator_current",
      "sotm_active", "vision_tag_count", "vision_feeding",
      "cancoder_position"
  });

  private final DiagnosticLogger hoodDiagLogger = new DiagnosticLogger("hood_diag", new String[] {
      "timestamp",
      "hood_setpoint", "hood_actual", "hood_error",
      "motor_voltage", "stator_current", "velocity_rps",
      "is_homed",
      "flywheel_rpm_target", "distance_to_target"
  });

  // QuestNav seeding state
  private boolean isQuestNavSeeded = false;

  // Physical reseed button on the robot (DIO, active-low for normally-open
  // button)
  private final DigitalInput reseedButtonInput = new DigitalInput(QuestNavConstants.RESEED_BUTTON_DIO_PORT);
  private final Trigger reseedButton = new Trigger(() -> !reseedButtonInput.get());

  // Auto setup
  private final SendableChooser<Command> autoChooser;

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
        new ShootCommand(flywheel, hood, conveyor, turret, shooterCalculator));
    NamedCommands.registerCommand("extendIntake", intakeActuator.extend());
    NamedCommands.registerCommand("agitateIntake", intakeActuator.agitate());
    NamedCommands.registerCommand("runIntake", intake.runCommand());

    autoChooser = AutoBuilder.buildAutoChooser();
    autoChooser.addOption("Drive Back & Shoot", buildDriveBackAndShootAuto());
    SmartDashboard.putData("Auto Mode", autoChooser);

    // Set turret default command - auto-aim with operator stick override
    turret.setDefaultCommand(turret.run(() -> {
      // ALWAYS update solver once per 20ms cycle for Newton predictions
      shooterCalculator.update();

      double stickX = operatorController.getLeftX();
      double stickY = operatorController.getLeftY();
      double magnitude = Math.hypot(stickX, stickY);

      double turretSetpoint;
      boolean sotmActive = false;

      if (magnitude > 0.15) {
        // Manual field-oriented override
        double fieldAngleRad = Math.atan2(-stickX, -stickY);
        double robotHeadingRad = drivetrain.getState().Pose.getRotation().getRadians();
        double turretAngleDeg = Math.toDegrees(fieldAngleRad - robotHeadingRad) % 360.0;
        if (turretAngleDeg > 180.0)
          turretAngleDeg -= 360.0;
        else if (turretAngleDeg <= -180.0)
          turretAngleDeg += 360.0;
        if (turretAngleDeg > TurretConstants.MAX_POSITION_DEGREES)
          turretAngleDeg -= 360.0;
        turretSetpoint = turretAngleDeg;
      } else {
        // Auto-aim at target
        var result = shooterCalculator.calculate();
        turretSetpoint = result.turretAngleDegrees();
        sotmActive = true;
      }

      turret.setPosition(turretSetpoint);

      // Diagnostic CSV logging
      if (turretDiagLogger.isOpen()) {
        var pose = drivetrain.getState().Pose;
        var speeds = drivetrain.getState().Speeds;
        var target = shooterCalculator.getLastTargetPosition();
        turretDiagLogger.logRow(
            Timer.getFPGATimestamp(),
            pose.getX(), pose.getY(), pose.getRotation().getDegrees(),
            speeds.vxMetersPerSecond, speeds.vyMetersPerSecond, speeds.omegaRadiansPerSecond,
            target.getX(), target.getY(),
            shooterCalculator.getLastDistanceMeters(),
            shooterCalculator.getLastRawAngleDegrees(),
            turretSetpoint,
            turretSetpoint,
            turret.getPositionDegrees(),
            turretSetpoint - turret.getPositionDegrees(),
            turret.getMotorVoltage(),
            turret.getStatorCurrent(),
            sotmActive ? shooterCalculator.getSotmConfidence() : 0.0,
            vision.getVisibleTagCount(),
            vision.isFeedingEnabled() ? 1.0 : 0.0,
            turret.getCANCoderPositionDegrees());
      }

      // Hood diagnostic CSV logging
      if (hoodDiagLogger.isOpen()) {
        double hoodSetpoint = hood.getTargetPositionDegrees();
        double hoodActual = hood.getCurrentPositionDegrees();
        hoodDiagLogger.logRow(
            Timer.getFPGATimestamp(),
            hoodSetpoint,
            hoodActual,
            hoodSetpoint - hoodActual,
            hood.getMotorVoltage(),
            hood.getStatorCurrent(),
            hood.getVelocityRPS(),
            hood.isHomed() ? 1.0 : 0.0,
            shooterCalculator.getFlywheelRPM(),
            shooterCalculator.getLastDistanceMeters());
      }
    }).withName("TurretAimWithOverride"));

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
    // Auto-extend intake actuator at the start of autonomous and teleop
    RobotModeTriggers.teleop().onTrue(intakeActuator.extend());

    // LEFT TRIGGER - Intake (held)
    // Runs the intake roller to acquire game pieces
    driverController.leftTrigger(0.5).whileTrue(intake.run(intake::runIntake).finallyDo(intake::stop));

    // RIGHT TRIGGER - Shoot (held)
    driverController.rightTrigger(0.5).whileTrue(
        new ShootCommand(flywheel, hood, conveyor, turret,
            shooterCalculator,
            drivetrain,
            this::getDriveRequest,
            this::isDriverActive,
            intakeActuator)
            .withName("Shoot"));

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
   * Configure QuestNav seeding. Polls for 2+ visible AprilTags while disabled,
   * then seeds QuestNav pose.
   * The physical reseed button (DIO) resets and re-seeds when 2+ tags visible.
   */
  private void questNavInitialization() {
    // Automatic initial seeding: poll every second while disabled until first seed
    Command initialSeedingCommand = Commands.sequence(
        Commands.sequence(
            Commands.waitSeconds(1.0),
            Commands.runOnce(() -> {
              if (!isQuestNavSeeded && vision.getVisibleTagCount() >= 2) {
                if (questNav.seedPoseFromVision()) {
                  isQuestNavSeeded = true;
                }
              }
            })).repeatedly().until(() -> isQuestNavSeeded))
        .ignoringDisable(true);
    RobotModeTriggers.disabled().onTrue(initialSeedingCommand);

    // One-time auto-seed while enabled: seed QuestNav if it hasn't been seeded yet
    // After seeding, QuestNav feeds drivetrain via addVisionMeasurement (soft
    // correction)
    // and vision also continues feeding when 2+ tags visible (redundancy)
    Command enabledAutoSeedCommand = Commands.run(() -> {
      if (!isQuestNavSeeded && vision.getVisibleTagCount() >= 2) {
        if (questNav.seedPoseFromVision()) {
          isQuestNavSeeded = true;
        }
      }
    }).ignoringDisable(false);
    RobotModeTriggers.teleop().whileTrue(enabledAutoSeedCommand);
    RobotModeTriggers.autonomous().whileTrue(enabledAutoSeedCommand);

    // Reseed button: only works while disabled so a mid-match press is ignored
    reseedButton.and(RobotModeTriggers.disabled()).onTrue(Commands.sequence(
        Commands.runOnce(() -> {
          isQuestNavSeeded = false;
          questNav.clearSeeded();
        }),
        // Wait for 2+ visible tags, then reseed
        Commands.waitUntil(() -> vision.getVisibleTagCount() >= 2),
        Commands.runOnce(() -> {
          if (questNav.seedPoseFromVision()) {
            isQuestNavSeeded = true;
          }
        })).ignoringDisable(true));
  }

  /**
   * Configure test mode bindings using RobotModeTriggers. These bindings are only
   * active when the robot is in test
   * mode.
   */
  private void configureTestBindings() {
    // Turret Calibration Mode
    // Right bumper toggles calibration mode - reads NetworkTables inputs
    // applies to hood/flywheel in real-time
    // TurretCalibrationCommand calibrationCmd = new TurretCalibrationCommand(
    // hood, flywheel, conveyor, intakeActuator,
    // () -> drivetrain.getState().Pose, shooterCalculator,
    // drivetrain,
    // this::getDriveRequest,
    // this::isDriverActive);

    // // Only allow toggling calibration mode while in test mode
    // RobotModeTriggers.test().and(driverController.rightBumper()).toggleOnTrue(calibrationCmd);

    // Swerve
    // Old commented WPILib sysId lines:
    // RobotModeTriggers.test().and(driverController.a()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kForward));
    // RobotModeTriggers.test().and(driverController.b()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kReverse));
    // RobotModeTriggers.test().and(driverController.x()).whileTrue(drivetrain.sysIdDynamic(Direction.kForward));
    // RobotModeTriggers.test().and(driverController.y()).whileTrue(drivetrain.sysIdDynamic(Direction.kReverse));

    // --- Drivetrain System Identification (for MATLAB) ---
    // RobotModeTriggers.test().and(driverController.a())
    // .toggleOnTrue(new DrivetrainSysIdCommand(drivetrain,
    // DrivetrainSysIdCommand.Routine.QUASISTATIC));
    // RobotModeTriggers.test().and(driverController.b())
    // .toggleOnTrue(new DrivetrainSysIdCommand(drivetrain,
    // DrivetrainSysIdCommand.Routine.STEPS));
    // RobotModeTriggers.test().and(driverController.y())
    // .toggleOnTrue(new DrivetrainSysIdCommand(drivetrain,
    // DrivetrainSysIdCommand.Routine.COASTDOWN));

    // --- Steer System Identification (for MATLAB) ---
    // RobotModeTriggers.test().and(operatorController.a())
    // .toggleOnTrue(new SteerSysIdCommand(drivetrain,
    // SteerSysIdCommand.Routine.QUASISTATIC));
    // RobotModeTriggers.test().and(operatorController.b())
    // .toggleOnTrue(new SteerSysIdCommand(drivetrain,
    // SteerSysIdCommand.Routine.STEPS));
    // RobotModeTriggers.test().and(operatorController.y())
    // .toggleOnTrue(new SteerSysIdCommand(drivetrain,
    // SteerSysIdCommand.Routine.COASTDOWN));

    // --- Conveyor ---
    // RobotModeTriggers.test().and(driverController.rightBumper()).whileTrue(conveyor.run(conveyor::runFeed));
    // RobotModeTriggers.test().and(driverController.leftBumper()).onTrue(Commands.runOnce(conveyor::stop,
    // conveyor));

    // --- Intake Actuator ---
    // RobotModeTriggers.test().and(driverController.a()).whileTrue(intakeActuator.extend());
    // RobotModeTriggers.test().and(driverController.b()).whileTrue(intakeActuator.retract());
    // RobotModeTriggers.test().and(driverController.x()).whileTrue(intakeActuator.agitate());

    // --- Intake ---
    // RobotModeTriggers.test().and(driverController.leftBumper()).whileTrue(intake.run(intake::runIntake));
    // RobotModeTriggers.test().and(driverController.x()).onTrue(Commands.runOnce(intake::stop,
    // intake));

    // --- Turret Manual Test ---
    // driverController.a()
    // .whileTrue(turret.run(turret::testDirectionPositive).finallyDo(() ->
    // turret.stop()));
    // driverController.b()
    // .whileTrue(turret.run(turret::testDirectionNegative).finallyDo(() ->
    // turret.stop()));
    // RobotModeTriggers.test().and(driverController.x())
    // .onTrue(Commands.runOnce(() -> turret.setPosition(0)));
    // RobotModeTriggers.test().and(driverController.y())
    // .onTrue(Commands.runOnce(() -> turret.nudge(-120)));
    // RobotModeTriggers.test().and(driverController.leftBumper()).onTrue(Commands.runOnce(turret::stop,
    // turret));

    // --- Turret Field Hold ---
    // RobotModeTriggers.test().and(driverController.rightBumper())
    // .whileTrue(turret.fieldHoldCommand(() ->
    // drivetrain.getState().Pose.getRotation().getDegrees()));
    // RobotModeTriggers.test().and(driverController.leftBumper()).onTrue(Commands.runOnce(turret::stop,
    // turret));

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

  /** Builds a field-centric drive request from driver joystick input. */
  private SwerveRequest getDriveRequest() {
    return drive
        .withVelocityX(-driverController.getLeftY() * MaxSpeed)
        .withVelocityY(-driverController.getLeftX() * MaxSpeed)
        .withRotationalRate(-driverController.getRightX() * MaxAngularRate);
  }

  /** Returns true when the driver is actively providing joystick input. */
  private boolean isDriverActive() {
    double stickMag = Math.hypot(driverController.getLeftX(),
        driverController.getLeftY());
    return stickMag > 0.1 || Math.abs(driverController.getRightX()) > 0.1;
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
        Commands.parallel(new ShootCommand(flywheel, hood, conveyor, turret, shooterCalculator),
            intakeActuator.agitate(),
            intake.runCommand()));
  }

  public Command getAutonomousCommand() {
    return autoChooser.getSelected();
  }

  public void startDiagnosticLogging() {
    turretDiagLogger.open();
    hoodDiagLogger.open();
  }

  public void stopDiagnosticLogging() {
    turretDiagLogger.close();
    hoodDiagLogger.close();
  }
}
