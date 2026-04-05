package frc.robot;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.FollowPathCommand;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants.OperatorConstants;
import frc.robot.Constants.QuestNavConstants;
import frc.robot.Constants.TurretConstants;
import frc.robot.commands.DriveCommands;
import frc.robot.commands.DumbShootCommand;
import frc.robot.commands.ShootCommand;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Flywheel;
import frc.robot.subsystems.Hood;
import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.IntakeActuator;
import frc.robot.subsystems.QuestNavSubsystem;
import frc.robot.subsystems.Turret;
import frc.robot.subsystems.Vision;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.GyroIO;
import frc.robot.subsystems.drive.GyroIOPigeon2;
import frc.robot.subsystems.drive.ModuleIO;
import frc.robot.subsystems.drive.ModuleIOSim;
import frc.robot.subsystems.drive.ModuleIOTalonFX;
import frc.robot.util.AimingCalculator;
import frc.robot.util.Telemetry;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

public class RobotContainer {

  // Subsystems
  private final Drive drive;
  private final Indexer indexer = new Indexer();
  private final Flywheel flywheel = new Flywheel();
  private final Hood hood = new Hood();
  private final Intake intake = new Intake();
  private final IntakeActuator intakeActuator = new IntakeActuator();
  private final Turret turret = new Turret();

  // Controllers
  private final CommandXboxController driverController =
      new CommandXboxController(OperatorConstants.DRIVER_CONTROLLER_PORT);
  private final CommandXboxController operatorController =
      new CommandXboxController(OperatorConstants.OPERATOR_CONTROLLER_PORT);

  // Swerve constants
  private double MaxSpeed = 1.0 * TunerConstants.kSpeedAt12Volts.in(MetersPerSecond);
  private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond);

  private final SwerveRequest.FieldCentric old_drive =
      new SwerveRequest.FieldCentric()
          .withDeadband(MaxSpeed * 0.1)
          .withRotationalDeadband(MaxAngularRate * 0.1)
          .withDriveRequestType(DriveRequestType.OpenLoopVoltage);

  // Swerve setup
  private final Telemetry logger = new Telemetry(MaxSpeed);
  public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();

  // Vision
  private Vision vision = null;
  private QuestNavSubsystem questNav = null;

  // Turret aiming calculator
  private AimingCalculator shooterCalculator = null;

  // QuestNav seeding state
  private boolean isQuestNavSeeded = false;

  // Physical reseed button on the robot
  private final DigitalInput reseedButtonInput =
      new DigitalInput(QuestNavConstants.RESEED_BUTTON_DIO_PORT);
  private final Trigger reseedButton = new Trigger(() -> !reseedButtonInput.get());

  // Auto setup
  private final LoggedDashboardChooser<Command> autoChooser;

  public RobotContainer() {
    DriverStation.silenceJoystickConnectionWarning(true);

    switch (Constants.currentMode) {
      case REAL:
        // Real robot, instantiate hardware IO implementations
        // ModuleIOTalonFX is intended for modules with TalonFX drive, TalonFX turn, and
        // a CANcoder
        drive =
            new Drive(
                new GyroIOPigeon2(),
                new ModuleIOTalonFX(TunerConstants.FrontLeft),
                new ModuleIOTalonFX(TunerConstants.FrontRight),
                new ModuleIOTalonFX(TunerConstants.BackLeft),
                new ModuleIOTalonFX(TunerConstants.BackRight));

        // The ModuleIOTalonFXS implementation provides an example implementation for
        // TalonFXS controller connected to a CANdi with a PWM encoder. The
        // implementations
        // of ModuleIOTalonFX, ModuleIOTalonFXS, and ModuleIOSpark (from the Spark
        // swerve
        // template) can be freely intermixed to support alternative hardware
        // arrangements.
        // Please see the AdvantageKit template documentation for more information:
        // https://docs.advantagekit.org/getting-started/template-projects/talonfx-swerve-template#custom-module-implementations
        //
        // drive =
        // new Drive(
        // new GyroIOPigeon2(),
        // new ModuleIOTalonFXS(TunerConstants.FrontLeft),
        // new ModuleIOTalonFXS(TunerConstants.FrontRight),
        // new ModuleIOTalonFXS(TunerConstants.BackLeft),
        // new ModuleIOTalonFXS(TunerConstants.BackRight));
        break;

      case SIM:
        // Sim robot, instantiate physics sim IO implementations
        drive =
            new Drive(
                new GyroIO() {},
                new ModuleIOSim(TunerConstants.FrontLeft),
                new ModuleIOSim(TunerConstants.FrontRight),
                new ModuleIOSim(TunerConstants.BackLeft),
                new ModuleIOSim(TunerConstants.BackRight));
        break;

      default:
        // Replayed robot, disable IO implementations
        drive =
            new Drive(
                new GyroIO() {},
                new ModuleIO() {},
                new ModuleIO() {},
                new ModuleIO() {},
                new ModuleIO() {});
        break;
    }

    // Set up auto routines
    autoChooser = new LoggedDashboardChooser<>("Auto Choices", AutoBuilder.buildAutoChooser());

    // Set up SysId routines
    autoChooser.addOption(
        "Drive Wheel Radius Characterization", DriveCommands.wheelRadiusCharacterization(drive));
    autoChooser.addOption(
        "Drive Simple FF Characterization", DriveCommands.feedforwardCharacterization(drive));
    autoChooser.addOption(
        "Drive SysId (Quasistatic Forward)",
        drive.sysIdQuasistatic(SysIdRoutine.Direction.kForward));
    autoChooser.addOption(
        "Drive SysId (Quasistatic Reverse)",
        drive.sysIdQuasistatic(SysIdRoutine.Direction.kReverse));
    autoChooser.addOption(
        "Drive SysId (Dynamic Forward)", drive.sysIdDynamic(SysIdRoutine.Direction.kForward));
    autoChooser.addOption(
        "Drive SysId (Dynamic Reverse)", drive.sysIdDynamic(SysIdRoutine.Direction.kReverse));

    // Configure the button bindings
    configureButtonBindings();

    // Initialize vision subsystems
    vision = new Vision(drivetrain::addVisionMeasurement);
    if (QuestNavConstants.ENABLED) {
      questNav = new QuestNavSubsystem(drivetrain, vision);
    }
    shooterCalculator =
        new AimingCalculator(() -> drivetrain.getState().Pose, () -> drivetrain.getState().Speeds);

    // Register named commands before building auto chooser
    NamedCommands.registerCommand(
        "shoot", new ShootCommand(flywheel, hood, indexer, turret, shooterCalculator));
    NamedCommands.registerCommand("extendIntake", intakeActuator.extend());
    NamedCommands.registerCommand("agitateIntake", intakeActuator.agitate());
    NamedCommands.registerCommand("runIntake", intake.runCommand());

    // autoChooser = AutoBuilder.buildAutoChooser();
    // autoChooser.addOption("Drive Back & Shoot", buildDriveBackAndShootAuto());
    // SmartDashboard.putData("Auto Mode", autoChooser);
    // SmartDashboard.putNumber("IntakeActuator/StepTestDutyCycle",
    // IntakeActuatorConstants.STEP_TEST_DUTY_CYCLE);

    // Set turret default command - auto-aim with operator stick override
    turret.setDefaultCommand(
        turret
            .run(
                () -> {
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
                    if (turretAngleDeg > 180.0) turretAngleDeg -= 360.0;
                    else if (turretAngleDeg <= -180.0) turretAngleDeg += 360.0;
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
                })
            .withName("TurretAimWithOverride"));

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

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    // Default command, normal field-relative drive
    drive.setDefaultCommand(
        DriveCommands.joystickDrive(
            drive,
            () -> -driverController.getLeftY(),
            () -> -driverController.getLeftX(),
            () -> -driverController.getRightX()));

    // Lock to 0° when A button is held
    driverController
        .a()
        .whileTrue(
            DriveCommands.joystickDriveAtAngle(
                drive,
                () -> -driverController.getLeftY(),
                () -> -driverController.getLeftX(),
                () -> Rotation2d.kZero));

    // Switch to X pattern when X button is pressed
    driverController.x().onTrue(Commands.runOnce(drive::stopWithX, drive));

    // Reset gyro to 0° when B button is pressed
    driverController
        .b()
        .onTrue(
            Commands.runOnce(
                    () ->
                        drive.setPose(
                            new Pose2d(drive.getPose().getTranslation(), Rotation2d.kZero)),
                    drive)
                .ignoringDisable(true));
  }

  private void configureDriverBindings() {
    // Auto-extend intake actuator at the start of autonomous and teleop
    RobotModeTriggers.teleop().onTrue(intakeActuator.extend());

    // LEFT TRIGGER - Intake (held)
    // Runs the intake roller to acquire game pieces
    driverController
        .leftTrigger(0.5)
        .whileTrue(intake.run(intake::runIntake).finallyDo(intake::stop));

    // RIGHT TRIGGER - Shoot (held)
    driverController
        .rightTrigger(0.5)
        .whileTrue(
            new ShootCommand(flywheel, hood, indexer, turret, shooterCalculator, intakeActuator)
                .withName("Shoot"));

    // LEFT BUMPER - Retract intake actuator (held)
    driverController.leftBumper().whileTrue(intakeActuator.retract());

    // RIGHT BUMPER - Dumb Shoot (held)
    driverController
        .rightBumper()
        .whileTrue(
            new DumbShootCommand(flywheel, hood, indexer, intakeActuator).withName("DumbShoot"));

    // DEFAULT COMMAND - Field-Centric Drive
    drivetrain.setDefaultCommand(drivetrain.applyRequest(this::getDriveRequest));

    // DISABLED MODE - Idle Request
    final var idle = new SwerveRequest.Idle();
    RobotModeTriggers.disabled()
        .whileTrue(drivetrain.applyRequest(() -> idle).ignoringDisable(true));
  }

  private void configureOperatorBindings() {}

  /**
   * Configure QuestNav seeding. Polls for 2+ visible AprilTags while disabled, then seeds QuestNav
   * pose. The physical reseed button (DIO) resets and re-seeds when 2+ tags visible.
   */
  private void questNavInitialization() {
    // Automatic initial seeding: poll every second while disabled until first seed
    Command initialSeedingCommand =
        Commands.sequence(
                Commands.sequence(
                        Commands.waitSeconds(1.0),
                        Commands.runOnce(
                            () -> {
                              if (!isQuestNavSeeded && vision.getVisibleTagCount() >= 2) {
                                if (questNav.seedPoseFromVision()) {
                                  isQuestNavSeeded = true;
                                }
                              }
                            }))
                    .repeatedly()
                    .until(() -> isQuestNavSeeded))
            .ignoringDisable(true);
    RobotModeTriggers.disabled().onTrue(initialSeedingCommand);

    // One-time auto-seed while enabled: seed QuestNav if it hasn't been seeded yet
    // After seeding, QuestNav feeds drivetrain via addVisionMeasurement (soft
    // correction)
    // and vision also continues feeding when 2+ tags visible (redundancy)
    Command enabledAutoSeedCommand =
        Commands.run(
                () -> {
                  if (!isQuestNavSeeded && vision.getVisibleTagCount() >= 2) {
                    if (questNav.seedPoseFromVision()) {
                      isQuestNavSeeded = true;
                    }
                  }
                })
            .ignoringDisable(false);
    RobotModeTriggers.teleop().whileTrue(enabledAutoSeedCommand);
    RobotModeTriggers.autonomous().whileTrue(enabledAutoSeedCommand);

    // Reseed button: only works while disabled so a mid-match press is ignored
    reseedButton
        .and(RobotModeTriggers.disabled())
        .onTrue(
            Commands.sequence(
                    Commands.runOnce(
                        () -> {
                          isQuestNavSeeded = false;
                          questNav.clearSeeded();
                        }),
                    // Wait for 2+ visible tags, then reseed
                    Commands.waitUntil(() -> vision.getVisibleTagCount() >= 2),
                    Commands.runOnce(
                        () -> {
                          if (questNav.seedPoseFromVision()) {
                            isQuestNavSeeded = true;
                          }
                        }))
                .ignoringDisable(true));
  }

  /**
   * Configure test mode bindings using RobotModeTriggers. These bindings are only active when the
   * robot is in test mode.
   */
  private void configureTestBindings() {
    // Only allow toggling calibration mode while in test mode
    // RobotModeTriggers.test().and(driverController.rightBumper()).toggleOnTrue(calibrationCmd);

    // --- Indexer ---
    // RobotModeTriggers.test().and(driverController.povUp()).whileTrue(indexer.feedCommand());
    // RobotModeTriggers.test().and(driverController.povDown()).onTrue(Commands.runOnce(indexer::stop,
    // indexer));

    // // --- Intake Actuator ---
    // RobotModeTriggers.test().and(driverController.a()).whileTrue(intakeActuator.extend());
    // RobotModeTriggers.test().and(driverController.b()).whileTrue(intakeActuator.retract());
    // RobotModeTriggers.test().and(driverController.x()).whileTrue(intakeActuator.agitate());

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

  /** Builds a field-centric drive request from driver joystick input. */
  private SwerveRequest getDriveRequest() {
    return old_drive
        .withVelocityX(-driverController.getLeftY() * MaxSpeed)
        .withVelocityY(-driverController.getLeftX() * MaxSpeed)
        .withRotationalRate(-driverController.getRightX() * MaxAngularRate);
  }

  private Command buildDriveBackAndShootAuto() {
    SwerveRequest.RobotCentric driveBack =
        new SwerveRequest.RobotCentric()
            .withVelocityX(-1.0) // backwards at 1 m/s
            .withVelocityY(0)
            .withRotationalRate(0);
    SwerveRequest.Idle stop = new SwerveRequest.Idle();

    return Commands.sequence(
        Commands.deadline(
            drivetrain.applyRequest(() -> driveBack).withTimeout(0.6), intakeActuator.extend()),
        drivetrain.applyRequest(() -> stop).withTimeout(0.02),
        Commands.parallel(
            new ShootCommand(flywheel, hood, indexer, turret, shooterCalculator),
            intakeActuator.agitate(),
            intake.runCommand()));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return autoChooser.get();
  }
}
