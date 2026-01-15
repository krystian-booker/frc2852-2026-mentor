package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.IntakeActuatorConstants;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.IntakeActuator;

public class ExtendWithSafetyCommand extends Command {

    private enum State {
        EXTENDING,
        RETRACTING
    }

    private final IntakeActuator intakeActuator;
    private final Intake intake;
    private State state;

    public ExtendWithSafetyCommand(IntakeActuator intakeActuator, Intake intake) {
        this.intakeActuator = intakeActuator;
        this.intake = intake;
        addRequirements(intakeActuator, intake);
    }

    @Override
    public void initialize() {
        state = State.EXTENDING;
        intakeActuator.setPosition(IntakeActuatorConstants.MAX_POSITION_DEGREES);
        intake.runIntake();
    }

    @Override
    public void execute() {
        double current = intakeActuator.getOutputCurrent();
        double position = intakeActuator.getCurrentPositionDegrees();

        boolean isExtended = position >= IntakeActuatorConstants.EXTENDED_POSITION_THRESHOLD_DEGREES;
        boolean isCurrentHigh = current > IntakeActuatorConstants.CURRENT_SPIKE_THRESHOLD_AMPS;

        switch (state) {
            case EXTENDING:
                // Keep intake running while extending
                intake.runIntake();

                if (isExtended && isCurrentHigh) {
                    // Current spike detected while extended - retract
                    state = State.RETRACTING;
                    intakeActuator.setPosition(IntakeActuatorConstants.MIN_POSITION_DEGREES);
                    intake.stop();
                    System.out.println("IntakeActuator: Safety retraction triggered! Current: " + current + "A");
                }
                break;

            case RETRACTING:
                // Keep intake stopped while retracting
                intake.stop();

                boolean isRetracted = position <= IntakeActuatorConstants.POSITION_TOLERANCE_DEGREES;
                if (isRetracted) {
                    // Reached retracted position, try extending again
                    state = State.EXTENDING;
                    intakeActuator.setPosition(IntakeActuatorConstants.MAX_POSITION_DEGREES);
                    intake.runIntake();
                }
                break;
        }
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        intake.stop();
    }

    @Override
    public String getName() {
        return "ExtendWithSafety";
    }
}
