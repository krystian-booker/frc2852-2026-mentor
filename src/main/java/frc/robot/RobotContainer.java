package frc.robot;

import frc.robot.Constants.OperatorConstants;
import frc.robot.subsystems.Flywheel;
import frc.robot.subsystems.Hood;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.IntakeActuator;
import frc.robot.subsystems.Turret;
import com.ctre.phoenix6.SignalLogger;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;

public class RobotContainer {

  // Subsystems
  private final Flywheel flywheel = new Flywheel();
  private final Hood hood = new Hood();
  private final Intake intake = new Intake();
  private final IntakeActuator intakeActuator = new IntakeActuator();
  private final Turret turret = new Turret();

  // Controllers
  private final CommandXboxController driverController = new CommandXboxController(
      OperatorConstants.DRIVER_CONTROLLER_PORT);

  public RobotContainer() {
    if (DriverStation.isTest()) {
      configureTestBindings();
    } else {
      configureBindings();
    }
  }

  private void configureBindings() {
    driverController.leftBumper().toggleOnTrue(intakeActuator.extendWithSafety(intake));
  }

  /**
   * Configure SysId bindings
   *
   * Button mapping:
   * A - Quasistatic Forward
   * B - Quasistatic Reverse
   * X - Dynamic Forward
   * Y - Dynamic Reverse
   */
  private void configureTestBindings() {
    // Start CTRE SignalLogger for Phoenix 6 SysId
    SignalLogger.start();

    // Flywheel
    // driverController.a().whileTrue(flywheel.sysIdQuasistatic(SysIdRoutine.Direction.kForward));
    // driverController.b().whileTrue(flywheel.sysIdQuasistatic(SysIdRoutine.Direction.kReverse));
    // driverController.x().whileTrue(flywheel.sysIdDynamic(SysIdRoutine.Direction.kForward));
    // driverController.y().whileTrue(flywheel.sysIdDynamic(SysIdRoutine.Direction.kReverse));

    // Hood
    // driverController.a().whileTrue(hood.sysIdQuasistatic(SysIdRoutine.Direction.kForward));
    // driverController.b().whileTrue(hood.sysIdQuasistatic(SysIdRoutine.Direction.kReverse));
    // driverController.x().whileTrue(hood.sysIdDynamic(SysIdRoutine.Direction.kForward));
    // driverController.y().whileTrue(hood.sysIdDynamic(SysIdRoutine.Direction.kReverse));

    // Turret
    // driverController.a().whileTrue(turret.sysIdQuasistatic(SysIdRoutine.Direction.kForward));
    // driverController.b().whileTrue(turret.sysIdQuasistatic(SysIdRoutine.Direction.kReverse));
    // driverController.x().whileTrue(turret.sysIdDynamic(SysIdRoutine.Direction.kForward));
    // driverController.y().whileTrue(turret.sysIdDynamic(SysIdRoutine.Direction.kReverse));

    // Intake
    // driverController.a().whileTrue(intake.sysIdQuasistatic(SysIdRoutine.Direction.kForward));
    // driverController.b().whileTrue(intake.sysIdQuasistatic(SysIdRoutine.Direction.kReverse));
    // driverController.x().whileTrue(intake.sysIdDynamic(SysIdRoutine.Direction.kForward));
    // driverController.y().whileTrue(intake.sysIdDynamic(SysIdRoutine.Direction.kReverse));

    // Intake Actuator
    // driverController.a().whileTrue(intakeActuator.sysIdQuasistatic(SysIdRoutine.Direction.kForward));
    // driverController.b().whileTrue(intakeActuator.sysIdQuasistatic(SysIdRoutine.Direction.kReverse));
    // driverController.x().whileTrue(intakeActuator.sysIdDynamic(SysIdRoutine.Direction.kForward));
    // driverController.y().whileTrue(intakeActuator.sysIdDynamic(SysIdRoutine.Direction.kReverse));
  }

  public Command getAutonomousCommand() {
    return null;
  }
}
