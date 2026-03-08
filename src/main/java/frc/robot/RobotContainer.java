package frc.robot;

import frc.robot.Constants.OperatorConstants;
import frc.robot.Constants.QuestNavConstants;
import frc.robot.commands.FlywheelAutoTuneCommand;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.Conveyor;
import frc.robot.subsystems.Flywheel;
import frc.robot.subsystems.Hood;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.IntakeActuator;
import frc.robot.subsystems.Turret;
import frc.robot.subsystems.Climb;
import frc.robot.subsystems.LED;
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
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.QuestNavSubsystem;

public class RobotContainer {

  // Subsystems
  // private final Conveyor conveyor = new Conveyor();
  private final Flywheel flywheel = new Flywheel();
  // private final Hood hood = new Hood();
  // private final Intake intake = new Intake();
  // private final IntakeActuator intakeActuator = new IntakeActuator();
  // private final Turret turret = new Turret();
  // private final Climb climb = new Climb();
  // private final LED led = new LED();

  // Controllers
  private final CommandXboxController driverController = new CommandXboxController(
      OperatorConstants.DRIVER_CONTROLLER_PORT);
  // private final CommandXboxController operatorController = new CommandXboxController(
  // OperatorConstants.OPERATOR_CONTROLLER_PORT);

  // Swerve constants
  private double MaxSpeed = 1.0 * TunerConstants.kSpeedAt12Volts.in(MetersPerSecond);
  private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond);

  private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
      .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1)
      .withDriveRequestType(DriveRequestType.OpenLoopVoltage);
  private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
  private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();

  // Swerve setup
  private final Telemetry logger = new Telemetry(MaxSpeed);
  public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();

  // Vision
  private final Vision vision = null;
  private final QuestNavSubsystem questNav = null;

  // Turret aiming calculator
  private final TurretAimingCalculator turretAimingCalculator = null;

  // QuestNav seeding state
  private boolean isQuestNavSeeded = false;
  private Pose2d seededPose = null;

  // Manual reseed mode state
  private boolean reseedModeActive = false;

  // Auto setup
  private final SendableChooser<Command> autoChooser;

  public RobotContainer() {
    autoChooser = AutoBuilder.buildAutoChooser("Tests");
    SmartDashboard.putData("Auto Mode", autoChooser);

    // Initialize vision subsystems (after drivetrain)
    // vision = new Vision(drivetrain::addVisionMeasurement);
    // questNav = new QuestNavSubsystem(drivetrain, vision);

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
    // intake.setDefaultCommand(intake.run(intake::runIntake));

    // Start CTRE SignalLogger once so all SysId tests go to a single file
    // SignalLogger.setPath("/home/lvuser/logs/");
    // SignalLogger.start();

    // Configure normal bindings (always available)
    // configureBindings();
    configureSwerveBindings();
    configureClimbBindings();

    // Configure test mode bindings (activated when entering test mode)
    configureTestBindings();

    // Configure disabled mode QuestNav seeding
    // configureDisabledBindings();

    // Warmup PathPlanner to avoid Java pauses
    CommandScheduler.getInstance().schedule(FollowPathCommand.warmupCommand());

    // Telemetry setup
    drivetrain.registerTelemetry(logger::telemeterize);

  }

  private void configureBindings() {
    // Auto-extend intake actuator at the start of autonomous and teleop
    // RobotModeTriggers.autonomous().onTrue(intakeActuator.extend());
    // RobotModeTriggers.teleop().onTrue(intakeActuator.extend());

    // RIGHT TRIGGER - Shoot (held)
    // Spins up flywheel and sets hood from LUT, then feeds when ready
    // driverController.rightTrigger(0.5).whileTrue(
    // new ShootCommand(flywheel, hood, conveyor, intakeActuator, turret, turretAimingCalculator)
    // .withName("Shoot"));
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
            .applyRequest(() -> drive.withVelocityX(driverController.getLeftY() * MaxSpeed)
                .withVelocityY(driverController.getLeftX() * MaxSpeed)
                .withRotationalRate(-driverController.getRightX() * MaxAngularRate)));

    // DISABLED MODE - Idle Request
    // Ensures the configured neutral mode (coast) is applied to
    // drive motors while the robot is disabled
    final var idle = new SwerveRequest.Idle();
    RobotModeTriggers.disabled().whileTrue(
        drivetrain.applyRequest(() -> idle).ignoringDisable(true));

    // A BUTTON - Brake Mode
    // Locks all 4 swerve modules into an X-pattern to prevent robot movement
    // driverController.a().whileTrue(drivetrain.applyRequest(() -> brake));

    // B BUTTON - Point Wheels
    // Points all wheels in the direction of the left stick without driving
    // Useful for aligning wheels before moving in a specific direction
    // driverController.b().whileTrue(drivetrain
    // .applyRequest(() -> point
    // .withModuleDirection(new Rotation2d(-driverController.getLeftY(), -driverController.getLeftX()))));

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

    // BACK BUTTON - Enter Reseed Mode
    // Enters manual reseed mode and sets LEDs to RED
    // driverController.back().onTrue(Commands.runOnce(() -> {
    // reseedModeActive = true;
    // led.setPattern(LED.Pattern.RED);
    // }));

    // START BUTTON - Execute Reseed (if conditions met)
    // Only seeds if reseed mode is active AND 2+ tags are visible
    // driverController.start().onTrue(Commands.either(
    // // If reseed mode active and 2+ tags visible: seed and double-flash
    // Commands.sequence(
    // Commands.runOnce(() -> {
    // questNav.seedPoseFromVision();
    // reseedModeActive = false;
    // }),
    // led.doubleFlashAndOffCommand(LED.Pattern.GREEN)),
    // // Otherwise do nothing
    // Commands.none(),
    // () -> reseedModeActive && vision.getVisibleTagCount() >= 2));

    // Tag count monitoring for reseed mode
    // When reseed mode is active and 2+ tags become visible, set LEDs GREEN
    // new Trigger(() -> reseedModeActive && vision.getVisibleTagCount() >= 2)
    // .onTrue(led.setPatternCommand(LED.Pattern.GREEN));

    // When reseed mode is active and tags drop below 2, set LEDs back to RED
    // new Trigger(() -> reseedModeActive && vision.getVisibleTagCount() < 2)
    // .onTrue(led.setPatternCommand(LED.Pattern.RED));
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
              // led.setPattern(LED.Pattern.RED);
              vision.setFeedingEnabled(true);
            }
          }

          // Attempt seeding if not seeded and 2+ tags visible
          if (!isQuestNavSeeded && vision.getVisibleTagCount() >= 2) {
            if (questNav.seedPoseFromVision()) {
              isQuestNavSeeded = true;
              seededPose = questNav.getLatestPose();
              // led.setPattern(LED.Pattern.GREEN);
              vision.setFeedingEnabled(false);
            }
          }
        })).repeatedly().ignoringDisable(true);

    RobotModeTriggers.disabled().whileTrue(seedingCommand);

    // Turn off LEDs when auto or teleop starts
    // RobotModeTriggers.autonomous().onTrue(led.setPatternCommand(LED.Pattern.BLACK));
    // RobotModeTriggers.teleop().onTrue(led.setPatternCommand(LED.Pattern.BLACK));
  }

  private void configureClimbBindings() {
    // Operator D-pad controls for climb positions (teleop)
    // operatorController.povUp()
    // .onTrue(Commands.runOnce(() -> climb.setPosition(ClimbConstants.FULL_EXTENSION_POSITION), climb));
    // operatorController.povDown()
    // .onTrue(Commands.runOnce(() -> climb.setPosition(ClimbConstants.FULLY_DOWN), climb));
    // operatorController.povLeft()
    // .onTrue(Commands.runOnce(() -> climb.setPosition(ClimbConstants.CLIMB_LIFT_POSITION), climb));

    // Manual jog controls for finding setpoints
    // Hold right bumper to move up, hold left bumper to move down
    // Read position from SmartDashboard "Climb/Position Rotations"
    // operatorController.rightBumper()
    // .whileTrue(climb.run(() -> climb.manualMove(12.0)).finallyDo(() -> climb.stop()));
    // operatorController.leftBumper()
    // .whileTrue(climb.run(() -> climb.manualMove(-12.0)).finallyDo(() -> climb.stop()));
  }

  /**
   * Configure test mode bindings using RobotModeTriggers. These bindings are only active when the robot is in test
   * mode.
   *
   * Button mapping: A - Quasistatic Forward B - Quasistatic Reverse X - Dynamic Forward Y - Dynamic Reverse Right
   * Bumper - Toggle Turret Calibration Mode
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

    // ===== MANUAL SUBSYSTEM TEST BINDINGS =====

    // --- Hood ---
    // Step 1: Move hood by hand, watch dashboard values (no buttons needed)
    // Step 2: Test motor direction - hold A for +1V, hold B for -1V (releases = neutral)
    // Watch dashboard: +voltage should INCREASE position. If not, flip motor inversion.
    // Step 3: After direction is confirmed, use X to nudge +5deg, Y to nudge -5deg
    // LB: Emergency stop
    // RobotModeTriggers.test().and(driverController.a())
    // .whileTrue(hood.run(hood::testDirectionPositive).finallyDo(() -> hood.setNeutral()));
    // RobotModeTriggers.test().and(driverController.b())
    // .whileTrue(hood.run(hood::testDirectionNegative).finallyDo(() -> hood.setNeutral()));
    // RobotModeTriggers.test().and(driverController.x())
    // .onTrue(hood.runOnce(() -> hood.nudge(5)));
    // RobotModeTriggers.test().and(driverController.y())
    // .onTrue(hood.runOnce(() -> hood.nudge(-5)));
    // RobotModeTriggers.test().and(driverController.leftBumper()).onTrue(Commands.runOnce(hood::setNeutral, hood));

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
    // A: Feed | B: Reverse | X: Floor only | Y: Takeup only | LB: Stop all
    // RobotModeTriggers.test().and(driverController.rightBumper()).whileTrue(conveyor.run(conveyor::runFeed));
    // RobotModeTriggers.test().and(driverController.leftBumper()).onTrue(Commands.runOnce(conveyor::stop, conveyor));

    // --- Intake Actuator ---
    // A: Go to SmartDashboard position | B: Retract | X: Agitate | Y: Stop
    // SmartDashboard.putNumber("IntakeActuatorSet Position", 0.0);
    // RobotModeTriggers.test().and(driverController.a()).whileTrue(intakeActuator.extend());
    // RobotModeTriggers.test().and(driverController.b()).whileTrue(intakeActuator.retract());
    // RobotModeTriggers.test().and(driverController.x()).whileTrue(intakeActuator.agitate());

    // --- Intake ---
    // A: Run intake | B: Run outtake | X: Stop
    // RobotModeTriggers.test().and(driverController.leftBumper()).whileTrue(intake.run(intake::runIntake));
    // RobotModeTriggers.test().and(driverController.x()).onTrue(Commands.runOnce(intake::stop, intake));

    // --- Climb (operator controller in test mode) ---
    // A: Hold for +1V forward | B: Hold for -1V reverse
    // X: Nudge +1 rotation | Y: Nudge -1 rotation
    // LB: Emergency stop
    // RobotModeTriggers.test().and(operatorController.a())
    // .whileTrue(climb.run(climb::testDirectionPositive).finallyDo(() -> climb.stop()));
    // RobotModeTriggers.test().and(operatorController.b())
    // .whileTrue(climb.run(climb::testDirectionNegative).finallyDo(() -> climb.stop()));
    // RobotModeTriggers.test().and(operatorController.x())
    // .onTrue(Commands.runOnce(() -> climb.nudge(1.0), climb));
    // RobotModeTriggers.test().and(operatorController.y())
    // .onTrue(Commands.runOnce(() -> climb.nudge(-1.0), climb));
    // RobotModeTriggers.test().and(operatorController.leftBumper())
    // .onTrue(Commands.runOnce(climb::stop, climb));

    // --- Turret Manual Test ---
    // A: Hold for +1V forward | B: Hold for -1V reverse
    // X: Nudge +90° | Y: Nudge -90°
    // RobotModeTriggers.test().and(driverController.a())
    // .whileTrue(turret.run(turret::testDirectionPositive).finallyDo(() -> turret.stop()));
    // RobotModeTriggers.test().and(driverController.b())
    // .whileTrue(turret.run(turret::testDirectionNegative).finallyDo(() -> turret.stop()));
    // RobotModeTriggers.test().and(driverController.x())
    // .onTrue(Commands.runOnce(() -> turret.nudge(120)));
    // RobotModeTriggers.test().and(driverController.y())
    // .onTrue(Commands.runOnce(() -> turret.nudge(-120)));
    // RobotModeTriggers.test().and(driverController.leftBumper()).onTrue(Commands.runOnce(turret::stop, turret));

    // --- Turret Field Hold ---
    // RB: Hold turret at a fixed field-relative heading while robot spins
    // RobotModeTriggers.test().and(driverController.rightBumper())
    // .whileTrue(turret.fieldHoldCommand(() -> drivetrain.getState().Pose.getRotation().getDegrees()));
    // RobotModeTriggers.test().and(driverController.leftBumper()).onTrue(Commands.runOnce(turret::stop, turret));

    // --- Flywheel Auto-Tune ---
    // BACK: Toggle flywheel auto-tune command
    RobotModeTriggers.test().and(driverController.back()).toggleOnTrue(new FlywheelAutoTuneCommand(flywheel));

    // --- Flywheel ---
    // A: 2000 RPM | B: 3500 RPM | X: 5000 RPM | Y: Stop
    RobotModeTriggers.test().and(driverController.a()).whileTrue(flywheel.run(() -> flywheel.setVelocity(2000)));
    RobotModeTriggers.test().and(driverController.b()).whileTrue(flywheel.run(() -> flywheel.setVelocity(3500)));
    RobotModeTriggers.test().and(driverController.x()).whileTrue(flywheel.run(() -> flywheel.setVelocity(5500)));
    RobotModeTriggers.test().and(driverController.y())
        .onTrue(Commands.runOnce(() -> flywheel.setVelocity(0), flywheel));

  }

  public Command getAutonomousCommand() {
    return autoChooser.getSelected();
  }
}
