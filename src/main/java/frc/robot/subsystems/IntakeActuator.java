package frc.robot.subsystems;

import com.revrobotics.PersistMode;
import com.revrobotics.REVLibError;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkFlexConfig;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants.CANIds;
import frc.robot.Constants.IntakeActuatorConstants;

public class IntakeActuator extends SubsystemBase {

    // Hardware
    private final SparkFlex motor;
    private final RelativeEncoder encoder;
    private final SparkClosedLoopController closedLoopController;

    public IntakeActuator() {
        motor = new SparkFlex(CANIds.INTAKE_ACTUATOR_MOTOR, MotorType.kBrushless);
        encoder = motor.getEncoder();
        closedLoopController = motor.getClosedLoopController();
        configureMotor();
    }

    private void configureMotor() {
        SparkFlexConfig config = new SparkFlexConfig();

        config.idleMode(IdleMode.kBrake);
        config.inverted(true);

        config.smartCurrentLimit(IntakeActuatorConstants.SMART_CURRENT_LIMIT);
        config.secondaryCurrentLimit(IntakeActuatorConstants.SECONDARY_CURRENT_LIMIT);

        config.encoder.positionConversionFactor(1.0 / IntakeActuatorConstants.GEAR_RATIO);
        config.encoder.velocityConversionFactor(1.0 / IntakeActuatorConstants.GEAR_RATIO / 60.0);

        config.closedLoop.feedbackSensor(FeedbackSensor.kPrimaryEncoder);
        config.closedLoop.pid(IntakeActuatorConstants.KP, IntakeActuatorConstants.KI, IntakeActuatorConstants.KD);
        config.closedLoop.outputRange(IntakeActuatorConstants.MIN_OUTPUT, IntakeActuatorConstants.MAX_OUTPUT);

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

        encoder.setPosition(0.0);
    }

    public void driveExtend() {
        closedLoopController.setSetpoint(IntakeActuatorConstants.EXTENDED_POSITION,
                SparkBase.ControlType.kPosition);
    }

    public void driveRetractAg() {
        closedLoopController.setSetpoint(IntakeActuatorConstants.RETRACTED_POSITION_AG,
                SparkBase.ControlType.kPosition);
    }

    public void driveRetract() {
        closedLoopController.setSetpoint(IntakeActuatorConstants.RETRACTED_POSITION,
                SparkBase.ControlType.kPosition);
    }

    public void driveExtendOpenLoop() {
        motor.set(IntakeActuatorConstants.EXTEND_DUTY_CYCLE);
    }

    public void driveRetractOpenLoop() {
        motor.set(IntakeActuatorConstants.RETRACT_DUTY_CYCLE);
    }

    public boolean isExtended() {
        return Math.abs(encoder.getPosition()
                - IntakeActuatorConstants.EXTENDED_POSITION) <= IntakeActuatorConstants.POSITION_TOLERANCE_ROTATIONS;
    }

    public boolean isRetracted() {
        return Math.abs(encoder.getPosition()
                - IntakeActuatorConstants.RETRACTED_POSITION) <= IntakeActuatorConstants.POSITION_TOLERANCE_ROTATIONS;
    }

    public double getPosition() {
        return encoder.getPosition();
    }

    public void stop() {
        motor.set(0);
    }

    public void resetEncoder() {
        encoder.setPosition(0.0);
    }

    // Imperative agitation API (for use inside execute() loops)

    private boolean agitateRetract;
    private long agitateTimer;

    /** Resets agitation state. Call once before starting agitation cycles. */
    public void resetAgitate() {
        agitateRetract = true;
        agitateTimer = System.currentTimeMillis();
    }

    /** Drives one cycle of agitation. Call each execute() iteration. */
    public void runAgitate() {
        double timeout = agitateRetract
                ? IntakeActuatorConstants.AGITATE_RETRACT_SECONDS
                : IntakeActuatorConstants.AGITATE_EXTEND_SECONDS;
        if (System.currentTimeMillis() - agitateTimer >= timeout * 1000) {
            agitateRetract = !agitateRetract;
            agitateTimer = System.currentTimeMillis();
        }
        if (agitateRetract) {
            driveRetractAg();
        } else {
            driveExtend();
        }
    }

    // Commands

    public Command extend() {
        return runOnce(this::driveExtend)
                .withName("IntakeActuator.extend");
    }

    public Command retract() {
        return runOnce(this::driveRetract)
                .withName("IntakeActuator.retract");
    }

    public Command agitate() {
        return runOnce(this::resetAgitate)
                .andThen(run(this::runAgitate))
                .finallyDo(this::driveExtend)
                .withName("IntakeActuator.agitate");
    }

    public Command stepTest() {
        return run(() -> motor.set(SmartDashboard.getNumber("IntakeActuator/StepTestDutyCycle",
                IntakeActuatorConstants.STEP_TEST_DUTY_CYCLE)))
                        .finallyDo(this::stop)
                        .withName("IntakeActuator.stepTest");
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("IntakeActuator/Position", encoder.getPosition());
        SmartDashboard.putNumber("IntakeActuator/Velocity", encoder.getVelocity());
        SmartDashboard.putNumber("IntakeActuator/AppliedOutput", motor.getAppliedOutput());
    }
}
