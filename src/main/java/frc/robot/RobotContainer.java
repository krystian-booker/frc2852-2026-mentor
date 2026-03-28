package frc.robot;

import frc.robot.Constants.ClimbConstants;
import frc.robot.Constants.OperatorConstants;
import frc.robot.Constants.QuestNavConstants;
import frc.robot.Constants.TurretConstants;
import frc.robot.commands.DumbShootCommand;
import frc.robot.commands.HoodTestSequenceCommand;
import frc.robot.commands.ShootCommand;
import frc.robot.commands.TurretCalibrationCommand;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.Conveyor;
import frc.robot.subsystems.Flywheel;
import frc.robot.subsystems.Hood;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.LED;
import frc.robot.subsystems.IntakeActuator;
import frc.robot.subsystems.Turret;
import frc.robot.subsystems.Climb;
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
  private final Climb climb = new Climb();
  private final LED led = new LED();

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
        var result = shooterCalculator.calculateSOTM();
        turretSetpoint = result.turretAngleDegrees();
        sotmActive = result.sotmActive();
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
            sotmActive ? 1.0 : 0.0,
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
    // CommandScheduler.getInstance().schedule(PathfindingCommand.warmupCommand());

    // Telemetry setup
    drivetrain.registerTelemetry(logger::telemeterize);

  }

  private void configureDriverBindings() {
    // Auto-extend intake actuator at the start of autonomous and teleop
    // RobotModeTriggers.teleop().onTrue(intakeActuator.extend());

    // LEFT TRIGGER - Intake (held)
    // Runs the intake roller to acquire game pieces
    driverController.leftTrigger(0.5).whileTrue(intake.run(intake::runIntake).finallyDo(intake::stop));

    // RIGHT TRIGGER - Shoot (held)
    // Spins up flywheel and sets hood from LUT, then feeds when ready
    // Locks wheels in X-brake while shooting unless driver is actively driving
    driverController.rightTrigger(0.5).whileTrue(
        new ShootCommand(flywheel, hood, conveyor, turret,
            shooterCalculator,
            drivetrain,
            this::getDriveRequest,
            this::isDriverActive,
            intakeActuator)
                .withName("Shoot"));

    // DEFAULT COMMAND - Field-Centric Drive
    // Note that X is defined as forward according to WPILib convention,
    // and Y is defined as to the left according to WPILib convention.
    // Left Stick: Controls translation (forward/backward and left/right)
    // Right Stick X: Controls rotation (counterclockwise is positive)
    // The negatives account for controller axis inversion
    drivetrain.setDefaultCommand(
        drivetrain.applyRequest(this::getDriveRequest));

    // DISABLED MODE - Idle Request
    // Ensures the configured neutral mode (coast) is applied to
    // drive motors while the robot is disabled
    final var idle = new SwerveRequest.Idle();
    RobotModeTriggers.disabled().whileTrue(
        drivetrain.applyRequest(() -> idle).ignoringDisable(true));

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

    // RIGHT TRIGGER - Dumb Shot (held)
    // Fixed hood angle and flywheel RPM, drivetrain holds stationary
    operatorController.rightTrigger(0.5).whileTrue(
        new DumbShootCommand(flywheel, hood, conveyor, intakeActuator)
            .withName("DumbShoot"));

    // Intake actuator controls
    operatorController.a().onTrue(intakeActuator.extend());
    operatorController.b().onTrue(intakeActuator.retract());
    operatorController.x().onTrue(intakeActuator.agitate());

    // START - Reseed QuestNav from vision
    // Drive to where 2+ AprilTags are visible, then press start to reseed position
    if (QuestNavConstants.ENABLED) {
      operatorController.start().onTrue(Commands.sequence(
          Commands.runOnce(() -> {
            isQuestNavSeeded = false;
            questNav.clearSeeded();
            vision.setFeedingEnabled(true);
          }),
          led.setPatternCommand(LED.Pattern.RED),
          Commands.waitUntil(() -> vision.getVisibleTagCount() >= 2),
          Commands.runOnce(() -> {
            if (questNav.seedPoseFromVision()) {
              isQuestNavSeeded = true;
              vision.setFeedingEnabled(false);
            }
          }),
          led.setPatternCommand(LED.Pattern.GREEN)).ignoringDisable(true));
    }

    // Climb manual jog controls for zeroing
    RobotModeTriggers.test().and(operatorController.rightBumper()
        .whileTrue(climb.run(() -> climb.manualMove(12.0)).finallyDo(() -> climb.stop())));
    RobotModeTriggers.test().and(operatorController.leftBumper()
        .whileTrue(climb.run(() -> climb.manualMove(-12.0)).finallyDo(() -> climb.stop())));
  }

  /**
   * Configure QuestNav seeding with LED feedback. LEDs start OFF, turn RED when searching for tags, and GREEN once
   * seeded. The physical reseed button (DIO) resets to RED and re-seeds when 2+ tags visible.
   */
  private void questNavInitialization() {
    // Automatic initial seeding: poll every second while disabled until first seed
    Command initialSeedingCommand = Commands.sequence(
        // Turn RED to indicate searching for tags
        led.setPatternCommand(LED.Pattern.RED),
        Commands.sequence(
            Commands.waitSeconds(1.0),
            Commands.runOnce(() -> {
              if (!isQuestNavSeeded && vision.getVisibleTagCount() >= 2) {
                if (questNav.seedPoseFromVision()) {
                  isQuestNavSeeded = true;
                  vision.setFeedingEnabled(false);
                }
              }
            })).repeatedly().until(() -> isQuestNavSeeded),
        // Turn GREEN once seeded
        led.setPatternCommand(LED.Pattern.GREEN)).ignoringDisable(true);
    RobotModeTriggers.disabled().onTrue(initialSeedingCommand);

    // Reseed button: only works while disabled so a mid-match press is ignored
    reseedButton.and(RobotModeTriggers.disabled()).onTrue(Commands.sequence(
        Commands.runOnce(() -> {
          isQuestNavSeeded = false;
          questNav.clearSeeded();
          vision.setFeedingEnabled(true);
        }),
        // Turn RED while waiting for tags
        led.setPatternCommand(LED.Pattern.RED),
        // Wait for 2+ visible tags, then reseed
        Commands.waitUntil(() -> vision.getVisibleTagCount() >= 2),
        Commands.runOnce(() -> {
          if (questNav.seedPoseFromVision()) {
            isQuestNavSeeded = true;
            vision.setFeedingEnabled(false);
          }
        }),
        // Turn GREEN once re-seeded
        led.setPatternCommand(LED.Pattern.GREEN)).ignoringDisable(true));
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
    // hood, flywheel, conveyor, intakeActuator,
    // () -> drivetrain.getState().Pose, shooterCalculator,
    // drivetrain,
    // this::getDriveRequest,
    // this::isDriverActive);

    // // Only allow toggling calibration mode while in test mode
    // RobotModeTriggers.test().and(driverController.rightBumper()).toggleOnTrue(calibrationCmd);

    // Swerve
    // RobotModeTriggers.test().and(driverController.a()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kForward));
    // RobotModeTriggers.test().and(driverController.b()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kReverse));
    // RobotModeTriggers.test().and(driverController.x()).whileTrue(drivetrain.sysIdDynamic(Direction.kForward));
    // RobotModeTriggers.test().and(driverController.y()).whileTrue(drivetrain.sysIdDynamic(Direction.kReverse));

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

    // --- Climb ---
    // RobotModeTriggers.test().and(driverController.a())
    // .whileTrue(climb.run(climb::testDirectionPositive).finallyDo(() ->
    // climb.stop()));
    // RobotModeTriggers.test().and(driverController.b())
    // .whileTrue(climb.run(climb::testDirectionNegative).finallyDo(() ->
    // climb.stop()));
    // RobotModeTriggers.test().and(driverController.x())
    // .onTrue(Commands.runOnce(() -> climb.nudge(1.0), climb));
    // RobotModeTriggers.test().and(driverController.y())
    // .onTrue(Commands.runOnce(() -> climb.nudge(-1.0), climb));
    // RobotModeTriggers.test().and(driverController.leftBumper())
    // .onTrue(Commands.runOnce(climb::stop, climb));

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

    // --- Flywheel Auto-Tune ---
    // RobotModeTriggers.test().and(driverController.back()).toggleOnTrue(new
    // FlywheelAutoTuneCommand(flywheel));

    // --- Flywheel ---
    // RobotModeTriggers.test().and(driverController.a()).whileTrue(flywheel.run(()
    // ->
    // flywheel.setVelocity(2000)));
    // RobotModeTriggers.test().and(driverController.b()).whileTrue(flywheel.run(()
    // ->
    // flywheel.setVelocity(3500)));
    // RobotModeTriggers.test().and(driverController.x()).whileTrue(flywheel.run(()
    // -> flywheel.setVelocity(6000)));
    // RobotModeTriggers.test().and(driverController.y())
    // .onTrue(Commands.runOnce(() -> flywheel.setVelocity(0), flywheel));

    // --- Hood Test Sequence ---
    // Operator start button: zeros hood then runs through full range for diagnostic capture
    RobotModeTriggers.test().and(driverController.start())
        .onTrue(HoodTestSequenceCommand.create(hood));

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
        // Drive backwards for 0.5m (at 1 m/s ≈ 0.6s with acceleration margin) and extend intake
        // deadline() ends when the first command (drive) finishes, interrupting extend()
        // The intake motor keeps its setpoint so it continues extending during the shoot phase
        Commands.deadline(
            drivetrain.applyRequest(() -> driveBack)
                .withTimeout(0.6),
            intakeActuator.extend()),
        // Stop drivetrain before shooting
        drivetrain.applyRequest(() -> stop).withTimeout(0.02),
        // Shoot (runs until auto period ends)
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
