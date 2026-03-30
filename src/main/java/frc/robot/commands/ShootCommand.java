package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Conveyor;
import frc.robot.subsystems.Flywheel;
import frc.robot.subsystems.Hood;
import frc.robot.subsystems.IntakeActuator;
import frc.robot.subsystems.Turret;
import frc.robot.util.AimingCalculator;

/**
 * Shooting command that coordinates flywheel, hood, conveyor, and intake
 * actuator.
 *
 * <p>
 * Phase 1 (Spin-up): Sets flywheel RPM and hood angle from the lookup table
 * based on robot position. Continuously
 * updates as the robot moves. Waits for flywheel to reach setpoint.
 *
 * <p>
 * Phase 2 (Feeding): Once the flywheel is at speed, hood is at position, and
 * turret is aimed, runs the conveyor and
 * agitates the intake actuator to feed game pieces.
 *
 * <p>
 * This command never finishes on its own - it runs until interrupted (button
 * released). The turret is NOT required by
 * this command; its default aim command continues independently.
 */
public class ShootCommand extends Command {

    private final Flywheel flywheel;
    private final Hood hood;
    private final Conveyor conveyor;
    private final AimingCalculator aimingCalculator;
    private final IntakeActuator intakeActuator;

    private boolean isFeeding;
    private boolean waitingForExtend;

    /**
     * Teleop constructor - includes intake actuator.
     */
    public ShootCommand(
            Flywheel flywheel,
            Hood hood,
            Conveyor conveyor,
            Turret turret,
            AimingCalculator aimingCalculator,
            IntakeActuator intakeActuator) {
        this.flywheel = flywheel;
        this.hood = hood;
        this.conveyor = conveyor;
        this.aimingCalculator = aimingCalculator;
        this.intakeActuator = intakeActuator;

        addRequirements(flywheel, hood, conveyor, intakeActuator);
    }

    /** Auto constructor - no intake actuator. */
    public ShootCommand(
            Flywheel flywheel,
            Hood hood,
            Conveyor conveyor,
            Turret turret,
            AimingCalculator aimingCalculator) {
        this.flywheel = flywheel;
        this.hood = hood;
        this.conveyor = conveyor;
        this.aimingCalculator = aimingCalculator;
        this.intakeActuator = null;

        addRequirements(flywheel, hood, conveyor);
    }

    @Override
    public void initialize() {
        isFeeding = false;
        waitingForExtend = false;
        if (intakeActuator != null) {
            intakeActuator.resetAgitate();
        }
    }

    @Override
    public void execute() {
        // Get flywheel RPM and hood angle from the SOTM solver
        double targetRPM = aimingCalculator.getFlywheelRPM();
        double targetHoodAngle = aimingCalculator.getHoodAngle();

        flywheel.setVelocity(targetRPM);
        hood.setPosition(targetHoodAngle);

        // Check if all conditions are met to begin feeding
        boolean flywheelReady = flywheel.atSetpoint();
        boolean hoodReady = hood.atPosition();
        boolean turretReady = true; // turret.isAtPosition();

        if (flywheelReady && hoodReady && turretReady) {
            isFeeding = true;
            conveyor.runFeed();
        } else {
            if (isFeeding) {
                conveyor.stop();
            }
            isFeeding = false;
        }

        // Intake actuator control (teleop only)
        if (intakeActuator != null) {
            if (waitingForExtend) {
                intakeActuator.driveExtend();
                if (intakeActuator.isExtended()) {
                    waitingForExtend = false;
                }
            } else {
                intakeActuator.runAgitate();
            }
        }

        // Telemetry
        // SmartDashboard.putBoolean("Shoot/Feeding", isFeeding);
        // SmartDashboard.putBoolean("Shoot/FlywheelReady", flywheelReady);
        // SmartDashboard.putBoolean("Shoot/HoodReady", hoodReady);
        // SmartDashboard.putBoolean("Shoot/TurretReady", turretReady);
        // SmartDashboard.putNumber("Shoot/TargetRPM", targetRPM);
        // SmartDashboard.putNumber("Shoot/TargetHoodAngle", targetHoodAngle);
    }

    @Override
    public void end(boolean interrupted) {
        flywheel.setVelocity(0);
        conveyor.stop();
        if (intakeActuator != null) {
            intakeActuator.driveExtend();
        }
        isFeeding = false;
    }

    @Override
    public boolean isFinished() {
        return false;
    }

}
