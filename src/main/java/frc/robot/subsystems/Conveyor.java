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
import frc.robot.Constants.ConveyorConstants;

public class Conveyor extends SubsystemBase {

    // Hardware
    private final SparkFlex floorMotor;
    private final SparkFlex takeupMotor;

    private final RelativeEncoder floorEncoder;
    private final RelativeEncoder takeupEncoder;

    public Conveyor() {
        // Initialize hardware
        floorMotor = new SparkFlex(CANIds.CONVEYOR_FLOOR_MOTOR, MotorType.kBrushless);
        takeupMotor = new SparkFlex(CANIds.CONVEYOR_TAKE_UP_MOTOR, MotorType.kBrushless);

        floorEncoder = floorMotor.getEncoder();
        takeupEncoder = takeupMotor.getEncoder();

        // Configure motors
        configureMotor(floorMotor, "Floor", false);
        configureMotor(takeupMotor, "Takeup", false);
    }

    private void configureMotor(SparkFlex motor, String name, boolean inverted) {
        SparkFlexConfig config = new SparkFlexConfig();

        // Motor output
        config.idleMode(IdleMode.kCoast);
        config.inverted(inverted);

        // Current limits - high limits for sustained high-speed operation
        config.smartCurrentLimit(ConveyorConstants.SMART_CURRENT_LIMIT);
        config.secondaryCurrentLimit(ConveyorConstants.SECONDARY_CURRENT_LIMIT);

        // Encoder configuration - convert to mechanism rotations
        config.encoder.positionConversionFactor(1.0 / ConveyorConstants.GEAR_RATIO);
        config.encoder.velocityConversionFactor(1.0 / ConveyorConstants.GEAR_RATIO / 60.0);

        // Apply configuration with retry
        REVLibError error = REVLibError.kError;
        for (int i = 0; i < 5; i++) {
            error = motor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
            if (error == REVLibError.kOk) {
                break;
            }
        }
        if (error != REVLibError.kOk) {
            System.err.println("Failed to configure Conveyor " + name + " motor: " + error);
        }
    }

    /**
     * Run all conveyor motors at feed speed to move balls to the shooter.
     */
    public void runFeed() {
        floorMotor.set(ConveyorConstants.FEED_SPEED);
        takeupMotor.set(ConveyorConstants.FEED_SPEED);
    }

    /**
     * Reverse all conveyor motors to clear jams.
     */
    public void runReverse() {
        floorMotor.set(ConveyorConstants.REVERSE_SPEED);
        takeupMotor.set(ConveyorConstants.REVERSE_SPEED);
    }

    /**
     * Run only the floor rollers
     */
    public void runFloor() {
        floorMotor.set(ConveyorConstants.FEED_SPEED);
    }

    /**
     * Run only the takeup motor
     */
    public void runTakeup() {
        takeupMotor.set(ConveyorConstants.FEED_SPEED);
    }

    /**
     * Stop all conveyor motors.
     */
    public void stop() {
        floorMotor.setVoltage(0);
        takeupMotor.setVoltage(0);
    }

    /**
     * Stop only the floor motor.
     */
    public void stopFloor() {
        floorMotor.setVoltage(0);
    }

    /**
     * Stop only the takeup motor.
     */
    public void stopTakeup() {
        takeupMotor.setVoltage(0);
    }

    public double getFloorVelocityRPS() {
        return floorEncoder.getVelocity();
    }

    public double getTakeupVelocityRPS() {
        return takeupEncoder.getVelocity();
    }

    @Override
    public void periodic() {
        // SmartDashboard.putNumber("Conveyor/Floor Velocity RPS", getFloorVelocityRPS());
        // SmartDashboard.putNumber("Conveyor/Takeup Velocity RPS", getTakeupVelocityRPS());
        // SmartDashboard.putNumber("Conveyor/Floor Output", floorMotor.getAppliedOutput());
        // SmartDashboard.putNumber("Conveyor/Takeup Output", takeupMotor.getAppliedOutput());
    }
}
