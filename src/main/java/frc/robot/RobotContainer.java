// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.OperatorConstants;
import frc.robot.subsystems.Flywheel;
import frc.robot.subsystems.Hood;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {

  // Subsystems
  private final Flywheel flywheel = new Flywheel();
  private final Hood hood = new Hood();

  // Controllers
  private final CommandXboxController driverController = new CommandXboxController(OperatorConstants.DRIVER_CONTROLLER_PORT);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the trigger bindings
    configureBindings();
  }


  private void configureBindings() {
    // SysId characterization for flywheel (use in test mode)
    // Quasistatic: slow voltage ramp to measure kS and kV
    // Dynamic: step voltage to measure kA
    driverController.a().whileTrue(flywheel.sysIdQuasistatic(SysIdRoutine.Direction.kForward));
    driverController.b().whileTrue(flywheel.sysIdQuasistatic(SysIdRoutine.Direction.kReverse));
    driverController.x().whileTrue(flywheel.sysIdDynamic(SysIdRoutine.Direction.kForward));
    driverController.y().whileTrue(flywheel.sysIdDynamic(SysIdRoutine.Direction.kReverse));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    return null;
  }
}
