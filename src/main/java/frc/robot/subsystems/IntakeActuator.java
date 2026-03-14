package frc.robot.subsystems;

import com.revrobotics.PersistMode;
import com.revrobotics.REVLibError;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkFlexConfig;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants.CANIds;
import frc.robot.Constants.IntakeActuatorConstants;

public class IntakeActuator extends SubsystemBase {

    public enum ActuatorState {
        IDLE, EXTENDING, EXTENDED, RETRACTING, RETRACTED
    }

    // Hardware
    private final SparkFlex motor;

    // State
    private ActuatorState state = ActuatorState.IDLE;
    private final Timer moveTimer = new Timer();
    private final Timer settledTimer = new Timer();
    private int stallCurrentCount = 0;

    public IntakeActuator() {
        motor = new SparkFlex(CANIds.INTAKE_ACTUATOR_MOTOR, MotorType.kBrushless);
        configureMotor();
    }

    private void configureMotor() {
        SparkFlexConfig config = new SparkFlexConfig();

        config.idleMode(IdleMode.kBrake);
        config.inverted(true);

        config.smartCurrentLimit(IntakeActuatorConstants.SMART_CURRENT_LIMIT);
        config.secondaryCurrentLimit(IntakeActuatorConstants.SECONDARY_CURRENT_LIMIT);

        REVLibError error = REVLibError.kError;
        for (int i = 0; i < 5; i++) {
            error = motor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
            if (error == REVLibError.kOk) {
                break;
            }
        }
        if (error != REVLibError.kOk) {
            System.err.println("Failed to configure IntakeActuator motor: " + error);
        }
    }

    public void driveExtend() {
        if (state == ActuatorState.EXTENDING) {
            return;
        }
        motor.set(IntakeActuatorConstants.EXTEND_DUTY_CYCLE);
        state = ActuatorState.EXTENDING;
        stallCurrentCount = 0;
        moveTimer.restart();
    }

    public void driveRetract() {
        if (state == ActuatorState.RETRACTING) {
            return;
        }
        motor.set(IntakeActuatorConstants.RETRACT_DUTY_CYCLE);
        state = ActuatorState.RETRACTING;
        stallCurrentCount = 0;
        moveTimer.restart();
    }

    public void driveExtendOpenLoop() {
        motor.set(IntakeActuatorConstants.EXTEND_DUTY_CYCLE);
    }

    public void driveRetractOpenLoop() {
        motor.set(IntakeActuatorConstants.RETRACT_DUTY_CYCLE);
    }

    public boolean isExtended() {
        return state == ActuatorState.EXTENDED;
    }

    public boolean isRetracted() {
        return state == ActuatorState.RETRACTED;
    }

    public double getOutputCurrent() {
        return motor.getOutputCurrent();
    }

    public void stop() {
        motor.set(0);
        if (state != ActuatorState.EXTENDED && state != ActuatorState.RETRACTED) {
            state = ActuatorState.IDLE;
        }
    }

    // Commands

    public Command extend() {
        return runOnce(this::driveExtend)
                .andThen(Commands.idle(this).until(this::isExtended).withTimeout(2.0))
                .finallyDo(() -> {
                    if (!isExtended()) {
                        stop();
                    }
                })
                .withName("IntakeActuator.extend");
    }

    public Command retract() {
        return runOnce(this::driveRetract)
                .andThen(Commands.idle(this).until(this::isRetracted).withTimeout(2.0))
                .finallyDo(() -> {
                    if (!isRetracted()) {
                        stop();
                    }
                })
                .withName("IntakeActuator.retract");
    }

    public Command agitate() {
        return Commands.repeatingSequence(
                runOnce(this::driveRetract)
                        .andThen(Commands.idle(this).until(this::isRetracted))
                        .withTimeout(IntakeActuatorConstants.AGITATE_RETRACT_SECONDS),
                runOnce(this::driveExtend)
                        .andThen(Commands.idle(this).until(this::isExtended))
                        .withTimeout(IntakeActuatorConstants.AGITATE_EXTEND_SECONDS))
                .finallyDo(this::driveExtend)
                .withName("IntakeActuator.agitate");
    }

    @Override
    public void periodic() {
        // Stall detection state machine
        if (state == ActuatorState.EXTENDING || state == ActuatorState.RETRACTING) {
            if (moveTimer.hasElapsed(IntakeActuatorConstants.STALL_DETECTION_DELAY_SECONDS)) {
                if (getOutputCurrent() >= IntakeActuatorConstants.STALL_CURRENT_THRESHOLD_AMPS) {
                    stallCurrentCount++;
                } else {
                    stallCurrentCount = 0;
                }

                if (stallCurrentCount >= IntakeActuatorConstants.STALL_CURRENT_SAMPLE_COUNT) {
                    motor.set(0);
                    state = (state == ActuatorState.EXTENDING)
                            ? ActuatorState.EXTENDED
                            : ActuatorState.RETRACTED;
                    stallCurrentCount = 0;
                    settledTimer.restart();
                }
            }
        }

        // Position re-check: if settled at a hard stop for too long, re-drive to confirm position
        if (state == ActuatorState.EXTENDED
                && settledTimer.hasElapsed(IntakeActuatorConstants.POSITION_RECHECK_INTERVAL_SECONDS)) {
            driveExtend();
        } else if (state == ActuatorState.RETRACTED
                && settledTimer.hasElapsed(IntakeActuatorConstants.POSITION_RECHECK_INTERVAL_SECONDS)) {
            driveRetract();
        }

        SmartDashboard.putString("IntakeActuator/State", state.name());
        SmartDashboard.putNumber("IntakeActuator/Output Current", getOutputCurrent());
        SmartDashboard.putNumber("IntakeActuator/Stall Count", stallCurrentCount);
        SmartDashboard.putNumber("IntakeActuator/Applied Output", motor.getAppliedOutput());
    }
}
