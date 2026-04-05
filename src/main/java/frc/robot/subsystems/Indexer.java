package frc.robot.subsystems;

import com.revrobotics.PersistMode;
import com.revrobotics.REVLibError;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkFlexConfig;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants.CANIds;
import frc.robot.Constants.IndexerConstants;

public class Indexer extends SubsystemBase {

    // Hardware
    private final SparkFlex leaderMotor;
    private final SparkFlex followerOneMotor;
    private final SparkFlex followerTwoMotor;

    private final RelativeEncoder leaderEncoder;

    public Indexer() {
        // Initialize hardware
        leaderMotor = new SparkFlex(CANIds.INDEXER_LEADER_MOTOR, MotorType.kBrushless);
        followerOneMotor = new SparkFlex(CANIds.INDEXER_FOLLOWER_ONE_MOTOR, MotorType.kBrushless);
        followerTwoMotor = new SparkFlex(CANIds.INDEXER_FOLLOWER_TWO_MOTOR, MotorType.kBrushless);

        leaderEncoder = leaderMotor.getEncoder();

        // Configure motors
        configureLeaderMotor();
        configureFollowerMotor(followerOneMotor, "FollowerOne");
        configureFollowerMotor(followerTwoMotor, "FollowerTwo");
    }

    private void configureLeaderMotor() {
        SparkFlexConfig config = new SparkFlexConfig();

        config.idleMode(IdleMode.kBrake);
        config.inverted(false);

        config.smartCurrentLimit(IndexerConstants.SMART_CURRENT_LIMIT);
        config.secondaryCurrentLimit(IndexerConstants.SECONDARY_CURRENT_LIMIT);

        config.encoder.positionConversionFactor(1.0 / IndexerConstants.GEAR_RATIO);
        config.encoder.velocityConversionFactor(1.0 / IndexerConstants.GEAR_RATIO / 60.0);

        REVLibError error = REVLibError.kError;
        for (int i = 0; i < 5; i++) {
            error = leaderMotor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
            if (error == REVLibError.kOk) {
                break;
            }
        }
        if (error != REVLibError.kOk) {
            System.err.println("Failed to configure Indexer Leader motor: " + error);
        }
    }

    private void configureFollowerMotor(SparkFlex motor, String name) {
        SparkFlexConfig config = new SparkFlexConfig();

        config.idleMode(IdleMode.kBrake);
        config.follow(CANIds.INDEXER_LEADER_MOTOR, true);

        config.smartCurrentLimit(IndexerConstants.SMART_CURRENT_LIMIT);
        config.secondaryCurrentLimit(IndexerConstants.SECONDARY_CURRENT_LIMIT);

        REVLibError error = REVLibError.kError;
        for (int i = 0; i < 5; i++) {
            error = motor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
            if (error == REVLibError.kOk) {
                break;
            }
        }
        if (error != REVLibError.kOk) {
            System.err.println("Failed to configure Indexer " + name + " motor: " + error);
        }
    }

    /**
     * Run all indexer motors at feed speed to move balls to the shooter.
     */
    public void runFeed() {
        leaderMotor.set(IndexerConstants.FEED_SPEED);
    }

    /**
     * Reverse all indexer motors to clear jams.
     */
    public void runReverse() {
        leaderMotor.set(IndexerConstants.REVERSE_SPEED);
    }

    /**
     * Stop all indexer motors.
     */
    public void stop() {
        leaderMotor.setVoltage(0);
    }

    public double getVelocityRPS() {
        return leaderEncoder.getVelocity();
    }

    @Override
    public void periodic() {
        // SmartDashboard.putNumber("Indexer/Velocity RPS", getVelocityRPS());
        // SmartDashboard.putNumber("Indexer/Applied Output", leaderMotor.getAppliedOutput());
    }
}
