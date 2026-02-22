package frc.robot.subsystems;

import com.revrobotics.PersistMode;
import com.revrobotics.REVLibError;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkFlexConfig;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;

import frc.robot.Constants.CANIds;
import frc.robot.Constants.IntakeActuatorConstants;

import static edu.wpi.first.units.Units.*;

public class IntakeActuator extends SubsystemBase {

    // Hardware
    private final SparkFlex motor;
    private final SparkClosedLoopController closedLoopController;
    private final RelativeEncoder encoder;

    // SysId routine for characterization
    private final SysIdRoutine sysIdRoutine;

    // State
    private double targetPositionDegrees = 0.0;

    public IntakeActuator() {
        // Initialize hardware
        motor = new SparkFlex(CANIds.INTAKE_ACTUATOR_MOTOR, MotorType.kBrushless);
        closedLoopController = motor.getClosedLoopController();
        encoder = motor.getEncoder();

        // Configure motor
        configureMotor();

        // Zero encoder on startup
        zeroEncoder();

        // Configure SysId routine for characterization
        sysIdRoutine = new SysIdRoutine(
                new SysIdRoutine.Config(
                        null, // Use default ramp rate (1 V/s)
                        Volts.of(4), // Use 4V for position mechanism
                        null, // Use default timeout (10 s)
                        (state) -> SmartDashboard.putString("IntakeActuator/SysId State", state.toString())),
                new SysIdRoutine.Mechanism(
                        (voltage) -> motor.setVoltage(voltage.in(Volts)),
                        log -> {
                            log.motor("intake-acuator-state")
                                    .voltage(Volts.of(motor.getAppliedOutput() * motor.getBusVoltage()))
                                    .angularPosition(Rotations.of(encoder.getPosition()))
                                    .angularVelocity(RotationsPerSecond.of(encoder.getVelocity()));
                        },
                        this));
    }

    private void configureMotor() {
        SparkFlexConfig config = new SparkFlexConfig();

        // Motor output
        config.idleMode(IdleMode.kBrake);
        config.inverted(false);

        // Current limits
        config.smartCurrentLimit(IntakeActuatorConstants.SMART_CURRENT_LIMIT);
        config.secondaryCurrentLimit(IntakeActuatorConstants.SECONDARY_CURRENT_LIMIT);

        // Encoder configuration - convert to mechanism rotations
        config.encoder.positionConversionFactor(1.0 / IntakeActuatorConstants.GEAR_RATIO);
        config.encoder.velocityConversionFactor(1.0 / IntakeActuatorConstants.GEAR_RATIO / 60.0);

        // Closed-loop PID configuration
        config.closedLoop.p(IntakeActuatorConstants.P);
        config.closedLoop.i(IntakeActuatorConstants.I);
        config.closedLoop.d(IntakeActuatorConstants.D);
        config.closedLoop.outputRange(-1.0, 1.0);

        // Soft limits (in mechanism rotations)
        config.softLimit.forwardSoftLimitEnabled(true);
        config.softLimit.forwardSoftLimit((float) (IntakeActuatorConstants.MAX_POSITION_DEGREES / 360.0));
        config.softLimit.reverseSoftLimitEnabled(true);
        config.softLimit.reverseSoftLimit((float) (IntakeActuatorConstants.MIN_POSITION_DEGREES / 360.0));

        // Apply configuration with retry
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

    private void zeroEncoder() {
        REVLibError error = REVLibError.kError;
        for (int i = 0; i < 5; i++) {
            error = encoder.setPosition(0.0);
            if (error == REVLibError.kOk) {
                break;
            }
        }
        if (error != REVLibError.kOk) {
            System.err.println("Failed to zero IntakeActuator encoder: " + error);
        }
    }

    public void setPosition(double degrees) {
        targetPositionDegrees = clamp(degrees, IntakeActuatorConstants.MIN_POSITION_DEGREES,
                IntakeActuatorConstants.MAX_POSITION_DEGREES);
        double rotations = targetPositionDegrees / 360.0;
        closedLoopController.setSetpoint(rotations, ControlType.kPosition);
    }

    public boolean atPosition() {
        return Math.abs(getCurrentPositionDegrees()
                - targetPositionDegrees) < IntakeActuatorConstants.POSITION_TOLERANCE_DEGREES;
    }

    public double getCurrentPositionDegrees() {
        return encoder.getPosition() * 360.0;
    }

    public double getTargetPositionDegrees() {
        return targetPositionDegrees;
    }

    public double getOutputCurrent() {
        return motor.getOutputCurrent();
    }

    public boolean isExtended() {
        return getCurrentPositionDegrees() >= IntakeActuatorConstants.EXTENDED_POSITION_THRESHOLD_DEGREES;
    }

    public boolean isRetracted() {
        return getCurrentPositionDegrees() <= IntakeActuatorConstants.POSITION_TOLERANCE_DEGREES;
    }

    public void stop() {
        motor.setVoltage(0);
    }

    // Commands

    public Command retract() {
        return run(() -> setPosition(IntakeActuatorConstants.MIN_POSITION_DEGREES))
                .until(this::atPosition)
                .withName("IntakeActuator.retract");
    }

    public Command extend() {
        return run(() -> setPosition(IntakeActuatorConstants.MAX_POSITION_DEGREES))
                .until(this::atPosition)
                .withName("IntakeActuator.extend");
    }

    // SysId
    public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
        return sysIdRoutine.quasistatic(direction);
    }

    public Command sysIdDynamic(SysIdRoutine.Direction direction) {
        return sysIdRoutine.dynamic(direction);
    }

    // Helpers
    private double clamp(double value, double min, double max) {
        return Math.max(min, Math.min(max, value));
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("IntakeActuator/Position Degrees", getCurrentPositionDegrees());
        SmartDashboard.putNumber("IntakeActuator/Target Degrees", targetPositionDegrees);
        SmartDashboard.putNumber("IntakeActuator/Velocity", encoder.getVelocity());
        SmartDashboard.putBoolean("IntakeActuator/At Position", atPosition());
        SmartDashboard.putNumber("IntakeActuator/Applied Output", motor.getAppliedOutput());
        SmartDashboard.putNumber("IntakeActuator/Output Current", getOutputCurrent());
        SmartDashboard.putBoolean("IntakeActuator/Is Extended", isExtended());
    }
}
