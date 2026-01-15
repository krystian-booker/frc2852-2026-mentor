package frc.robot.subsystems;

import com.revrobotics.PersistMode;
import com.revrobotics.REVLibError;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkFlexConfig;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;

import frc.robot.Constants.CANIds;
import frc.robot.Constants.IntakeConstants;

import static edu.wpi.first.units.Units.*;

public class Intake extends SubsystemBase {

    // Hardware
    private final SparkFlex motor;
    private final RelativeEncoder encoder;

    // SysId routine for characterization
    private final SysIdRoutine sysIdRoutine;

    public Intake() {
        // Initialize hardware
        motor = new SparkFlex(CANIds.INTAKE_MOTOR, MotorType.kBrushless);
        encoder = motor.getEncoder();

        // Configure motor
        configureMotor();

        // Configure SysId routine for characterization
        sysIdRoutine = new SysIdRoutine(
                new SysIdRoutine.Config(
                        null, // Use default ramp rate (1 V/s)
                        Volts.of(7), // Use 7V for velocity mechanism
                        null, // Use default timeout (10 s)
                        (state) -> SmartDashboard.putString("Intake/SysId State", state.toString())),
                new SysIdRoutine.Mechanism(
                        (voltage) -> motor.setVoltage(voltage.in(Volts)),
                        log -> {
                            log.motor("intake")
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
        config.smartCurrentLimit(IntakeConstants.SMART_CURRENT_LIMIT);
        config.secondaryCurrentLimit(IntakeConstants.SECONDARY_CURRENT_LIMIT);

        // Encoder configuration - convert to mechanism rotations
        config.encoder.positionConversionFactor(1.0 / IntakeConstants.GEAR_RATIO);
        config.encoder.velocityConversionFactor(1.0 / IntakeConstants.GEAR_RATIO / 60.0);

        // Apply configuration with retry
        REVLibError error = REVLibError.kError;
        for (int i = 0; i < 5; i++) {
            error = motor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
            if (error == REVLibError.kOk) {
                break;
            }
        }
        if (error != REVLibError.kOk) {
            System.err.println("Failed to configure Intake motor: " + error);
        }
    }

    private void setSpeed(double speed) {
        motor.set(speed);
    }

    public void runIntake() {
        setSpeed(IntakeConstants.INTAKE_SPEED);
    }

    public void runOuttake() {
        setSpeed(IntakeConstants.OUTTAKE_SPEED);
    }

    public void stop() {
        motor.setVoltage(0);
    }

    public double getVelocityRPS() {
        return encoder.getVelocity();
    }

    public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
        return sysIdRoutine.quasistatic(direction);
    }

    public Command sysIdDynamic(SysIdRoutine.Direction direction) {
        return sysIdRoutine.dynamic(direction);
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Intake/Velocity RPS", getVelocityRPS());
        SmartDashboard.putNumber("Intake/Applied Output", motor.getAppliedOutput());
    }
}
