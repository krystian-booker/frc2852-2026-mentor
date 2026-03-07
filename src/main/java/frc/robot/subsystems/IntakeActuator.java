package frc.robot.subsystems;

import com.revrobotics.PersistMode;
import com.revrobotics.REVLibError;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkFlexConfig;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.AnalogPotentiometer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;

import frc.robot.Constants.CANIds;
import frc.robot.Constants.IntakeActuatorConstants;

import static edu.wpi.first.units.Units.*;

public class IntakeActuator extends SubsystemBase {

    // Hardware
    private final SparkFlex motor;
    private final AnalogPotentiometer potentiometer;

    // PID controller (runs on RoboRIO since feedback comes from external potentiometer)
    private final PIDController pidController;

    // SysId routine for characterization
    private final SysIdRoutine sysIdRoutine;

    // State
    private double targetPositionDistance = 0.0;
    private boolean pidEnabled = false;

    public IntakeActuator() {
        // Initialize hardware
        motor = new SparkFlex(CANIds.INTAKE_ACTUATOR_MOTOR, MotorType.kBrushless);

        // Initialize string potentiometer on RoboRIO analog input
        // AnalogPotentiometer maps normalized voltage (0-1) to: (fullRange * voltage/vRef) + offset
        potentiometer = new AnalogPotentiometer(
                IntakeActuatorConstants.POTENTIOMETER_CHANNEL,
                IntakeActuatorConstants.POT_FULL_RANGE,
                IntakeActuatorConstants.POT_OFFSET);

        // Initialize PID controller (runs on RoboRIO each periodic cycle)
        pidController = new PIDController(
                IntakeActuatorConstants.P,
                IntakeActuatorConstants.I,
                IntakeActuatorConstants.D);
        pidController.setTolerance(IntakeActuatorConstants.POSITION_TOLERANCE_DISTANCE);

        // Configure motor
        configureMotor();

        // Configure SysId routine for characterization (linear units for linear rail)
        sysIdRoutine = new SysIdRoutine(
                new SysIdRoutine.Config(
                        null, // Use default ramp rate (1 V/s)
                        Volts.of(4), // Use 4V for position mechanism
                        null, // Use default timeout (10 s)
                        (state) -> SmartDashboard.putString("IntakeActuator/SysId State", state.toString())),
                new SysIdRoutine.Mechanism(
                        (voltage) -> motor.setVoltage(voltage.in(Volts)),
                        log -> {
                            log.motor("intake-actuator")
                                    .voltage(Volts.of(motor.getAppliedOutput() * motor.getBusVoltage()))
                                    .linearPosition(Meters.of(getCurrentPosition()))
                                    .linearVelocity(MetersPerSecond.of(0));
                        },
                        this));
    }

    private void configureMotor() {
        SparkFlexConfig config = new SparkFlexConfig();

        // Motor output
        config.idleMode(IdleMode.kCoast);
        config.inverted(true);

        // Current limits
        config.smartCurrentLimit(IntakeActuatorConstants.SMART_CURRENT_LIMIT);
        config.secondaryCurrentLimit(IntakeActuatorConstants.SECONDARY_CURRENT_LIMIT);

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

    public void setPosition(double distance) {
        targetPositionDistance = distance;
        pidController.setSetpoint(targetPositionDistance);
        pidEnabled = true;
    }

    public boolean atPosition() {
        return pidController.atSetpoint();
    }

    public double getCurrentPosition() {
        return potentiometer.get();
    }

    public double getTargetPositionDistance() {
        return targetPositionDistance;
    }

    public double getOutputCurrent() {
        return motor.getOutputCurrent();
    }

    public boolean isExtended() {
        return getCurrentPosition() >= IntakeActuatorConstants.EXTENDED_POSITION_THRESHOLD_DISTANCE;
    }

    public boolean isRetracted() {
        return getCurrentPosition() <= IntakeActuatorConstants.POSITION_TOLERANCE_DISTANCE;
    }

    public void stop() {
        motor.setVoltage(0);
        pidEnabled = false;
        pidController.reset();
    }

    // Commands

    public Command retract() {
        return run(() -> setPosition(IntakeActuatorConstants.MIN_POSITION_DISTANCE))
                .until(this::atPosition)
                .withName("IntakeActuator.retract");
    }

    public Command extend() {
        return run(() -> setPosition(IntakeActuatorConstants.MAX_POSITION_DISTANCE))
                .until(this::atPosition)
                .withName("IntakeActuator.extend");
    }

    public Command agitate() {
        return Commands.repeatingSequence(
                run(() -> setPosition(IntakeActuatorConstants.MIN_POSITION_DISTANCE))
                        .withTimeout(IntakeActuatorConstants.AGITATE_RETRACT_SECONDS),
                run(() -> setPosition(IntakeActuatorConstants.MAX_POSITION_DISTANCE))
                        .withTimeout(IntakeActuatorConstants.AGITATE_EXTEND_SECONDS))
                .finallyDo(() -> setPosition(IntakeActuatorConstants.MAX_POSITION_DISTANCE))
                .withName("IntakeActuator.agitate");
    }

    // SysId
    public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
        return sysIdRoutine.quasistatic(direction);
    }

    public Command sysIdDynamic(SysIdRoutine.Direction direction) {
        return sysIdRoutine.dynamic(direction);
    }

    @Override
    public void periodic() {
        // Run PID loop when enabled - calculate motor output from potentiometer feedback
        if (pidEnabled) {
            double output = pidController.calculate(getCurrentPosition());
            output = MathUtil.clamp(output, -IntakeActuatorConstants.MAX_OUTPUT, IntakeActuatorConstants.MAX_OUTPUT);
            motor.setVoltage(output * 12.0);
        }

        SmartDashboard.putNumber("IntakeActuator/Position Distance", getCurrentPosition());
        SmartDashboard.putNumber("IntakeActuator/Target Distance", targetPositionDistance);
        SmartDashboard.putBoolean("IntakeActuator/At Position", atPosition());
        SmartDashboard.putNumber("IntakeActuator/Applied Output", motor.getAppliedOutput());
        SmartDashboard.putNumber("IntakeActuator/Output Current", getOutputCurrent());
        SmartDashboard.putBoolean("IntakeActuator/Is Extended", isExtended());
        SmartDashboard.putBoolean("IntakeActuator/PID Enabled", pidEnabled);
    }
}
