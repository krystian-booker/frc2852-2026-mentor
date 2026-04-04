package frc.robot.subsystems;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants;
import frc.robot.Constants.CANIds;
import frc.robot.Constants.IntakeConstants;

import static edu.wpi.first.units.Units.*;

public class Intake extends SubsystemBase {

    // Hardware
    private final TalonFX leaderMotor;
    private final TalonFX followerMotor;

    // Control requests
    private final VoltageOut voltageRequest;
    private final NeutralOut neutralRequest;

    // Status signals
    private final StatusSignal<AngularVelocity> leaderVelocity;

    public Intake() {
        // Initialize motors
        CANBus canBus = new CANBus(CANIds.CANIVORE);
        leaderMotor = new TalonFX(CANIds.INTAKE_LEADER_MOTOR, canBus);
        followerMotor = new TalonFX(CANIds.INTAKE_FOLLOWER_MOTOR, canBus);

        // Initialize control requests
        voltageRequest = new VoltageOut(0);
        neutralRequest = new NeutralOut();

        // Configure motors
        configureLeaderMotor();
        configureFollowerMotor();

        // Cache status signals
        leaderVelocity = leaderMotor.getVelocity();

        BaseStatusSignal.setUpdateFrequencyForAll(
                Constants.SIGNAL_UPDATE_FREQUENCY_HZ,
                leaderVelocity);

        // Optimize CAN bus utilization
        leaderMotor.optimizeBusUtilization();
        followerMotor.optimizeBusUtilization();
    }

    private void configureLeaderMotor() {
        TalonFXConfiguration config = new TalonFXConfiguration();

        config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        config.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
        config.Feedback.SensorToMechanismRatio = IntakeConstants.GEAR_RATIO;

        config.CurrentLimits.StatorCurrentLimitEnable = true;
        config.CurrentLimits.StatorCurrentLimit = IntakeConstants.STATOR_CURRENT_LIMIT;
        config.CurrentLimits.SupplyCurrentLimitEnable = true;
        config.CurrentLimits.SupplyCurrentLimit = IntakeConstants.SUPPLY_CURRENT_LIMIT;
        config.CurrentLimits.SupplyCurrentLowerLimit = IntakeConstants.SUPPLY_CURRENT_LOWER_LIMIT;
        config.CurrentLimits.SupplyCurrentLowerTime = IntakeConstants.SUPPLY_CURRENT_LOWER_TIME;

        // Apply with retry
        StatusCode status = StatusCode.StatusCodeNotInitialized;
        for (int i = 0; i < 5; i++) {
            status = leaderMotor.getConfigurator().apply(config);
            if (status.isOK()) {
                break;
            }
        }
        if (!status.isOK()) {
            System.err.println("Failed to configure intake leader motor: " + status);
        }
    }

    private void configureFollowerMotor() {
        TalonFXConfiguration config = new TalonFXConfiguration();
        config.MotorOutput.NeutralMode = NeutralModeValue.Brake;

        config.CurrentLimits.StatorCurrentLimitEnable = true;
        config.CurrentLimits.StatorCurrentLimit = IntakeConstants.STATOR_CURRENT_LIMIT;
        config.CurrentLimits.SupplyCurrentLimitEnable = true;
        config.CurrentLimits.SupplyCurrentLimit = IntakeConstants.SUPPLY_CURRENT_LIMIT;
        config.CurrentLimits.SupplyCurrentLowerLimit = IntakeConstants.SUPPLY_CURRENT_LOWER_LIMIT;
        config.CurrentLimits.SupplyCurrentLowerTime = IntakeConstants.SUPPLY_CURRENT_LOWER_TIME;

        StatusCode status = StatusCode.StatusCodeNotInitialized;
        for (int i = 0; i < 5; i++) {
            status = followerMotor.getConfigurator().apply(config);
            if (status.isOK()) {
                break;
            }
        }
        if (!status.isOK()) {
            System.err.println("Failed to configure intake follower motor: " + status);
        }

        // Follower spins in the opposite direction of the leader
        followerMotor.setControl(new Follower(CANIds.INTAKE_LEADER_MOTOR, MotorAlignmentValue.Opposed));
    }

    public void runIntake() {
        leaderMotor.setControl(voltageRequest.withOutput(IntakeConstants.INTAKE_VOLTAGE));
    }

    public void runOuttake() {
        leaderMotor.setControl(voltageRequest.withOutput(IntakeConstants.OUTTAKE_VOLTAGE));
    }

    public void stop() {
        leaderMotor.setControl(neutralRequest);
    }

    public double getVelocityRPS() {
        BaseStatusSignal.refreshAll(leaderVelocity);
        return leaderVelocity.getValue().in(RotationsPerSecond);
    }

    public Command runCommand() {
        return run(this::runIntake)
                .finallyDo(this::stop)
                .withName("Intake.run");
    }

    @Override
    public void periodic() {
        // SmartDashboard.putNumber("Intake/Velocity RPS", getVelocityRPS());
    }
}
