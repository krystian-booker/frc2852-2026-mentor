package frc.robot;

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
import frc.robot.subsystems.QuestNavSubsystem;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.GyroIO;
import frc.robot.subsystems.drive.GyroIOPigeon2;
import frc.robot.subsystems.drive.ModuleIO;
import frc.robot.subsystems.drive.ModuleIOSim;
import frc.robot.subsystems.drive.ModuleIOTalonFX;
import frc.robot.subsystems.flywheel.Flywheel;
import frc.robot.subsystems.flywheel.FlywheelIO;
import frc.robot.subsystems.flywheel.FlywheelIOSim;
import frc.robot.subsystems.flywheel.FlywheelIOTalonFX;
import frc.robot.subsystems.hood.Hood;
import frc.robot.subsystems.hood.HoodIO;
import frc.robot.subsystems.hood.HoodIOSim;
import frc.robot.subsystems.hood.HoodIOTalonFX;
import frc.robot.subsystems.indexer.Indexer;
import frc.robot.subsystems.indexer.IndexerIO;
import frc.robot.subsystems.indexer.IndexerIOSim;
import frc.robot.subsystems.indexer.IndexerIOSparkFlex;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intake.IntakeIO;
import frc.robot.subsystems.intake.IntakeIOSim;
import frc.robot.subsystems.intake.IntakeIOTalonFX;
import frc.robot.subsystems.intakeactuator.IntakeActuator;
import frc.robot.subsystems.intakeactuator.IntakeActuatorIO;
import frc.robot.subsystems.intakeactuator.IntakeActuatorIOSim;
import frc.robot.subsystems.intakeactuator.IntakeActuatorIOSparkFlex;
import frc.robot.subsystems.turret.Turret;
import frc.robot.subsystems.turret.TurretIO;
import frc.robot.subsystems.turret.TurretIOSim;
import frc.robot.subsystems.turret.TurretIOTalonFX;
import frc.robot.subsystems.vision.Vision;
import frc.robot.subsystems.vision.VisionConstants;
import frc.robot.subsystems.vision.VisionIO;
import frc.robot.subsystems.vision.VisionIOPhotonVision;
import frc.robot.subsystems.vision.VisionIOPhotonVisionSim;
import frc.robot.util.AimingCalculator;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

public class RobotContainer {

  // Subsystems
  private final Drive drive;
  private final Indexer indexer;
  private final Flywheel flywheel;
  private final Hood hood;
  private final Intake intake;
  private final IntakeActuator intakeActuator;
  private final Turret turret;

  // Controllers
  private final CommandXboxController driverController =
      new CommandXboxController(OperatorConstants.DRIVER_CONTROLLER_PORT);
  private final CommandXboxController operatorController =
      new CommandXboxController(OperatorConstants.OPERATOR_CONTROLLER_PORT);

  // Vision and QuestNav
  private final Vision vision;
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
        drive =
            new Drive(
                new GyroIOPigeon2(),
                new ModuleIOTalonFX(TunerConstants.FrontLeft),
                new ModuleIOTalonFX(TunerConstants.FrontRight),
                new ModuleIOTalonFX(TunerConstants.BackLeft),
                new ModuleIOTalonFX(TunerConstants.BackRight));
        flywheel = new Flywheel(new FlywheelIOTalonFX());
        hood = new Hood(new HoodIOTalonFX());
        turret = new Turret(new TurretIOTalonFX());
        intake = new Intake(new IntakeIOTalonFX());
        intakeActuator = new IntakeActuator(new IntakeActuatorIOSparkFlex());
        indexer = new Indexer(new IndexerIOSparkFlex());
        vision =
            new Vision(
                drive::addVisionMeasurement,
                new VisionIOPhotonVision(
                    VisionConstants.leftCameraName, VisionConstants.robotToLeftCamera),
                new VisionIOPhotonVision(
                    VisionConstants.rightCameraName, VisionConstants.robotToRightCamera));
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
        flywheel = new Flywheel(new FlywheelIOSim());
        hood = new Hood(new HoodIOSim());
        turret = new Turret(new TurretIOSim());
        intake = new Intake(new IntakeIOSim());
        intakeActuator = new IntakeActuator(new IntakeActuatorIOSim());
        indexer = new Indexer(new IndexerIOSim());
        vision =
            new Vision(
                drive::addVisionMeasurement,
                new VisionIOPhotonVisionSim(
                    VisionConstants.leftCameraName,
                    VisionConstants.robotToLeftCamera,
                    drive::getPose),
                new VisionIOPhotonVisionSim(
                    VisionConstants.rightCameraName,
                    VisionConstants.robotToRightCamera,
                    drive::getPose));
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
        flywheel = new Flywheel(new FlywheelIO() {});
        hood = new Hood(new HoodIO() {});
        turret = new Turret(new TurretIO() {});
        intake = new Intake(new IntakeIO() {});
        intakeActuator = new IntakeActuator(new IntakeActuatorIO() {});
        indexer = new Indexer(new IndexerIO() {});
        vision = new Vision(drive::addVisionMeasurement, new VisionIO() {}, new VisionIO() {});
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

    // Initialize QuestNav (disabled in SIM mode)
    if (QuestNavConstants.ENABLED && Constants.currentMode != Constants.Mode.SIM) {
      questNav = new QuestNavSubsystem(drive, vision);
    }
    shooterCalculator = new AimingCalculator(drive::getPose, drive::getChassisSpeeds);

    // Register named commands before building auto chooser
    NamedCommands.registerCommand(
        "shoot", new ShootCommand(flywheel, hood, indexer, turret, shooterCalculator));
    NamedCommands.registerCommand("extendIntake", intakeActuator.extend());
    NamedCommands.registerCommand("agitateIntake", intakeActuator.agitate());
    NamedCommands.registerCommand("runIntake", intake.runCommand());

    // Set turret default command - auto-aim with operator stick override
    turret.setDefaultCommand(
        turret
            .run(
                () -> {
                  // ALWAYS update solver once per 20ms cycle for Newton predictions
                  if (shooterCalculator != null) {
                    shooterCalculator.update();
                  }

                  double stickX = operatorController.getLeftX();
                  double stickY = operatorController.getLeftY();
                  double magnitude = Math.hypot(stickX, stickY);

                  double turretSetpoint = 0.0;

                  if (magnitude > 0.15) {
                    // Manual field-oriented override
                    double fieldAngleRad = Math.atan2(-stickX, -stickY);
                    double robotHeadingRad = drive.getPose().getRotation().getRadians();
                    double turretAngleDeg = Math.toDegrees(fieldAngleRad - robotHeadingRad) % 360.0;

                    if (turretAngleDeg > 180.0) turretAngleDeg -= 360.0;
                    else if (turretAngleDeg <= -180.0) turretAngleDeg += 360.0;
                    if (turretAngleDeg > TurretConstants.MAX_POSITION_DEGREES)
                      turretAngleDeg -= 360.0;
                    turretSetpoint = turretAngleDeg;
                  } else if (shooterCalculator != null) {
                    // Auto-aim at target
                    var result = shooterCalculator.calculate();
                    turretSetpoint = result.turretAngleDegrees();
                  }

                  turret.setPosition(turretSetpoint);
                })
            .withName("TurretAimWithOverride"));

    // Configure normal bindings (always available)
    configureDriverBindings();
    configureOperatorBindings();

    if (QuestNavConstants.ENABLED && Constants.currentMode != Constants.Mode.SIM) {
      questNavInitialization();
    }

    // Warmup PathPlanner to avoid Java pauses
    CommandScheduler.getInstance().schedule(FollowPathCommand.warmupCommand());
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
  }

  private void configureOperatorBindings() {}

  /**
   * Configure QuestNav seeding. Polls for 2+ visible AprilTags while disabled, then seeds QuestNav
   * pose. The physical reseed button (DIO) resets and re-seeds when 2+ tags visible.
   */
  private void questNavInitialization() {
    if (vision == null || questNav == null) {
      return;
    }

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
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return autoChooser.get();
  }
}
