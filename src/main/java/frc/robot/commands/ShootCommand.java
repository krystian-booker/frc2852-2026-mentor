package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;

import frc.robot.Constants.IntakeActuatorConstants;
import frc.robot.subsystems.Conveyor;
import frc.robot.subsystems.Flywheel;
import frc.robot.subsystems.Hood;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.IntakeActuator;
import frc.robot.subsystems.Turret;
import frc.robot.util.TurretAimingCalculator;

/**
 * Shooting command that coordinates flywheel, hood, conveyor, intake, and intake actuator.
 *
 * <p>Phase 1 (Spin-up): Sets flywheel RPM and hood angle from the lookup table based on robot
 * position. Continuously updates as the robot moves. Waits for flywheel to reach setpoint.
 *
 * <p>Phase 2 (Feeding): Once the flywheel is at speed, hood is at position, and turret is aimed,
 * runs the conveyor, intake roller, and intake actuator oscillation to feed game pieces.
 *
 * <p>This command never finishes on its own - it runs until interrupted (button released).
 * The turret is NOT required by this command; its default aim command continues independently.
 */
public class ShootCommand extends Command {

    private final Flywheel flywheel;
    private final Hood hood;
    private final Conveyor conveyor;
    private final Intake intake;
    private final IntakeActuator intakeActuator;
    private final Turret turret;
    private final TurretAimingCalculator aimingCalculator;

    private boolean isFeeding;

    public ShootCommand(
            Flywheel flywheel,
            Hood hood,
            Conveyor conveyor,
            Intake intake,
            IntakeActuator intakeActuator,
            Turret turret,
            TurretAimingCalculator aimingCalculator) {
        this.flywheel = flywheel;
        this.hood = hood;
        this.conveyor = conveyor;
        this.intake = intake;
        this.intakeActuator = intakeActuator;
        this.turret = turret;
        this.aimingCalculator = aimingCalculator;

        // Require all subsystems we command (not turret/intake - their default commands run independently)
        addRequirements(flywheel, hood, conveyor, intakeActuator);
    }

    @Override
    public void initialize() {
        isFeeding = false;
    }

    @Override
    public void execute() {
        // Continuously update flywheel RPM and hood angle from LUT
        double targetRPM = aimingCalculator.getFlywheelRPM();
        double targetHoodAngle = aimingCalculator.getHoodAngle();

        flywheel.setVelocity(targetRPM);
        hood.setPosition(targetHoodAngle);

        // Check if all conditions are met to begin feeding
        boolean flywheelReady = flywheel.atSetpoint();
        boolean hoodReady = hood.atPosition();
        boolean turretReady = turret.isAtPosition();

        if (flywheelReady && hoodReady && turretReady) {
            isFeeding = true;
            conveyor.runFeed();
            oscillateIntakeActuator();
        } else {
            if (isFeeding) {
                conveyor.stop();
            }
            isFeeding = false;
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
        isFeeding = false;
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    /**
     * Time-based oscillation for intake actuator agitation.
     * Alternates between agitate min and max positions every ~0.5 seconds.
     */
    private void oscillateIntakeActuator() {
        long cycleMs = System.currentTimeMillis() % 1000;
        if (cycleMs < 500) {
            intakeActuator.setPosition(IntakeActuatorConstants.AGITATE_MIN_DEGREES);
        } else {
            intakeActuator.setPosition(IntakeActuatorConstants.AGITATE_MAX_DEGREES);
        }
    }
}
