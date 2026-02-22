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
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants.CANIds;
import frc.robot.Constants.IntakeConstants;

public class Intake extends SubsystemBase {

    // Hardware
    private final SparkFlex motor;
    private final RelativeEncoder encoder;

    public Intake() {
        // Initialize hardware
        motor = new SparkFlex(CANIds.INTAKE_ROLLER_MOTOR, MotorType.kBrushless);
        encoder = motor.getEncoder();

        // Configure motor
        configureMotor();
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

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Intake/Velocity RPS", getVelocityRPS());
        SmartDashboard.putNumber("Intake/Applied Output", motor.getAppliedOutput());
    }
}
