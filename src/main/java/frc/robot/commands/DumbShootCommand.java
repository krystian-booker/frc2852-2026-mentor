package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;

import frc.robot.subsystems.Conveyor;
import frc.robot.subsystems.Flywheel;
import frc.robot.subsystems.Hood;
import frc.robot.subsystems.IntakeActuator;

/**
 * Fixed-parameter shoot command that uses hardcoded hood angle and flywheel RPM
 * instead of the aiming calculator. The
 * drivetrain is held stationary (no wiggle). Runs until interrupted.
 */
public class DumbShootCommand extends Command {

    private static final double HOOD_ANGLE_DEGREES = 15.0;
    private static final double FLYWHEEL_RPM = 2500.0;

    private final Flywheel flywheel;
    private final Hood hood;
    private final Conveyor conveyor;
    private final IntakeActuator intakeActuator;

    private boolean isFeeding;
    private boolean waitingForExtend;

    public DumbShootCommand(
            Flywheel flywheel,
            Hood hood,
            Conveyor conveyor,
            IntakeActuator intakeActuator) {
        this.flywheel = flywheel;
        this.hood = hood;
        this.conveyor = conveyor;
        this.intakeActuator = intakeActuator;

        addRequirements(flywheel, hood, conveyor, intakeActuator);
    }

    @Override
    public void initialize() {
        isFeeding = false;
        waitingForExtend = true;
        intakeActuator.resetAgitate();
    }

    @Override
    public void execute() {
        flywheel.setVelocity(FLYWHEEL_RPM);
        hood.setPosition(HOOD_ANGLE_DEGREES);

        // Feed when flywheel and hood are ready
        boolean flywheelReady = flywheel.atSetpoint();
        boolean hoodReady = hood.atPosition();

        if (flywheelReady && hoodReady) {
            isFeeding = true;
            conveyor.runFeed();
        } else {
            if (isFeeding) {
                conveyor.stop();
            }
            isFeeding = false;
        }

        // Intake actuator: extend first, then agitate
        if (waitingForExtend) {
            intakeActuator.driveExtend();
            if (intakeActuator.isExtended()) {
                waitingForExtend = false;
            }
        } else {
            intakeActuator.runAgitate();
        }
    }

    @Override
    public void end(boolean interrupted) {
        flywheel.setVelocity(0);
        conveyor.stop();
        intakeActuator.driveExtend();
        isFeeding = false;
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
