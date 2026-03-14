package frc.robot.subsystems;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants;
import frc.robot.Constants.CANIds;
import frc.robot.Constants.ClimbConstants;

import static edu.wpi.first.units.Units.*;

public class Climb extends SubsystemBase {

    // Hardware
    private final TalonFX motor;

    // Control requests
    private final MotionMagicVoltage positionRequest;
    private final NeutralOut neutralRequest;
    private final VoltageOut voltageRequest;

    // Status signals
    private final StatusSignal<Angle> motorPosition;
    private final StatusSignal<AngularVelocity> motorVelocity;
    private final StatusSignal<Current> statorCurrent;

    // State
    private double targetPositionRotations = 0.0;

    public Climb() {
        // Initialize hardware
        CANBus canBus = new CANBus(CANIds.CANIVORE);
        motor = new TalonFX(CANIds.CLIMB_MOTOR, canBus);

        // Initialize control requests
        positionRequest = new MotionMagicVoltage(0).withSlot(0);
        neutralRequest = new NeutralOut();
        voltageRequest = new VoltageOut(0);

        // Configure hardware
        configureMotor();
        motor.setPosition(0.0);

        // Cache status signals
        motorPosition = motor.getPosition();
        motorVelocity = motor.getVelocity();
        statorCurrent = motor.getStatorCurrent();

        BaseStatusSignal.setUpdateFrequencyForAll(
                Constants.SIGNAL_UPDATE_FREQUENCY_HZ,
                motorPosition,
                motorVelocity,
                statorCurrent,
                motor.getMotorVoltage());

        // Optimize CAN bus utilization
        motor.optimizeBusUtilization();
    }

    private void configureMotor() {
        TalonFXConfiguration config = new TalonFXConfiguration();

        // Motor output - Brake mode to hold robot weight
        config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        config.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;

        // Feedback - use rotor sensor, seeded to 0 on startup
        config.Feedback.SensorToMechanismRatio = ClimbConstants.GEAR_RATIO;

        // PID Gains (Slot 0)
        config.Slot0.kS = ClimbConstants.S;
        config.Slot0.kV = ClimbConstants.V;
        config.Slot0.kA = ClimbConstants.A;
        config.Slot0.kP = ClimbConstants.P;
        config.Slot0.kI = ClimbConstants.I;
        config.Slot0.kD = ClimbConstants.D;
        config.Slot0.GravityType = GravityTypeValue.Elevator_Static;
        config.Slot0.kG = ClimbConstants.G;

        // Motion Magic - already in rot/s units (no conversion needed)
        config.MotionMagic.MotionMagicCruiseVelocity = ClimbConstants.MOTION_MAGIC_CRUISE_VELOCITY;
        config.MotionMagic.MotionMagicAcceleration = ClimbConstants.MOTION_MAGIC_ACCELERATION;
        config.MotionMagic.MotionMagicJerk = ClimbConstants.MOTION_MAGIC_JERK;

        // Soft limits - already in rotations (no conversion needed)
        // config.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
        // config.SoftwareLimitSwitch.ForwardSoftLimitThreshold = ClimbConstants.MAX_POSITION;
        // config.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
        // config.SoftwareLimitSwitch.ReverseSoftLimitThreshold = ClimbConstants.MIN_POSITION;

        // Current limits
        config.CurrentLimits.StatorCurrentLimitEnable = true;
        config.CurrentLimits.StatorCurrentLimit = ClimbConstants.STATOR_CURRENT_LIMIT;
        config.CurrentLimits.SupplyCurrentLimitEnable = true;
        config.CurrentLimits.SupplyCurrentLimit = ClimbConstants.SUPPLY_CURRENT_LIMIT;
        config.CurrentLimits.SupplyCurrentLowerLimit = ClimbConstants.SUPPLY_CURRENT_LOWER_LIMIT;
        config.CurrentLimits.SupplyCurrentLowerTime = ClimbConstants.SUPPLY_CURRENT_LOWER_TIME;

        // Apply with retry
        StatusCode status = StatusCode.StatusCodeNotInitialized;
        for (int i = 0; i < 5; i++) {
            status = motor.getConfigurator().apply(config);
            if (status.isOK()) {
                break;
            }
        }
        if (!status.isOK()) {
            System.err.println("Failed to configure climb motor: " + status);
        }
    }

    public void setPosition(double rotations) {
        targetPositionRotations = clamp(rotations, ClimbConstants.MIN_POSITION, ClimbConstants.MAX_POSITION);
        motor.setControl(positionRequest.withPosition(targetPositionRotations));
    }

    public boolean atPosition() {
        double currentRotations = getCurrentPositionRotations();
        return Math.abs(currentRotations - targetPositionRotations) < ClimbConstants.POSITION_TOLERANCE;
    }

    public double getCurrentPositionRotations() {
        BaseStatusSignal.refreshAll(motorPosition);
        return motorPosition.getValue().in(Rotations);
    }

    public void stop() {
        motor.setControl(neutralRequest);
    }

    /**
     * Applies an arbitrary voltage for manual jogging. Use 2-3V for finding setpoints.
     */
    public void manualMove(double voltage) {
        motor.setControl(voltageRequest.withOutput(voltage));
    }

    /**
     * Applies a small positive voltage to test motor direction. Watch the dashboard: if position INCREASES, direction
     * is correct. If position DECREASES, flip motor inversion.
     */
    public void testDirectionPositive() {
        motor.setControl(voltageRequest.withOutput(1.0));
    }

    /**
     * Applies a small negative voltage to test motor direction.
     */
    public void testDirectionNegative() {
        motor.setControl(voltageRequest.withOutput(-1.0));
    }

    /**
     * Commands a small movement relative to current position for safe testing.
     */
    public void nudge(double deltaRotations) {
        double current = getCurrentPositionRotations();
        setPosition(current + deltaRotations);
    }

    /** Apply raw voltage output for feedforward characterization. */
    public void applyVoltage(double volts) {
        motor.setControl(voltageRequest.withOutput(volts));
    }

    /** Get mechanism velocity in rotations per second. */
    public double getVelocityRPS() {
        BaseStatusSignal.refreshAll(motorVelocity);
        return motorVelocity.getValue().in(RotationsPerSecond);
    }

    /** Get motor stator current in Amps. */
    public double getStatorCurrent() {
        BaseStatusSignal.refreshAll(statorCurrent);
        return statorCurrent.getValue().in(Amps);
    }

    /**
     * Hot-reload Slot0 gains and motion magic parameters using refresh+apply to preserve soft limits and current
     * limits.
     */
    public void applyTuningConfig(double kS, double kV, double kA, double kG, double kP, double kI, double kD,
            double cruise, double accel, double jerk) {
        var configurator = motor.getConfigurator();
        TalonFXConfiguration config = new TalonFXConfiguration();
        configurator.refresh(config);

        config.Slot0.kS = kS;
        config.Slot0.kV = kV;
        config.Slot0.kA = kA;
        config.Slot0.kG = kG;
        config.Slot0.kP = kP;
        config.Slot0.kI = kI;
        config.Slot0.kD = kD;
        config.MotionMagic.MotionMagicCruiseVelocity = cruise;
        config.MotionMagic.MotionMagicAcceleration = accel;
        config.MotionMagic.MotionMagicJerk = jerk;

        StatusCode status = configurator.apply(config);
        if (!status.isOK()) {
            System.err.println("Failed to apply climb tuning config: " + status);
        }
    }

    /** Restore original Constants.java values by re-running configureMotor(). */
    public void restoreDefaultConfig() {
        configureMotor();
    }

    private double clamp(double value, double min, double max) {
        return Math.max(min, Math.min(max, value));
    }

    @Override
    public void periodic() {
        // double position = getCurrentPositionRotations();
        // SmartDashboard.putNumber("Climb/Position Rotations", position);
        // SmartDashboard.putNumber("Climb/Target Rotations", targetPositionRotations);
        // SmartDashboard.putNumber("Climb/Motor Stator Current",
        // motor.getStatorCurrent().refresh().getValue().in(Amps));
        // SmartDashboard.putNumber("Climb/Motor Voltage", motor.getMotorVoltage().refresh().getValue().in(Volts));
        // SmartDashboard.putBoolean("Climb/At Position", atPosition());
    }
}
