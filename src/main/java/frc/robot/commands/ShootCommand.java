package frc.robot.commands;

import java.util.function.BooleanSupplier;
import java.util.function.Supplier;

import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;

import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Conveyor;
import frc.robot.subsystems.Flywheel;
import frc.robot.subsystems.Hood;
import frc.robot.subsystems.IntakeActuator;
import frc.robot.subsystems.Turret;
import frc.robot.util.TurretAimingCalculator;

/**
 * Shooting command that coordinates flywheel, hood, conveyor, and intake actuator.
 *
 * <p>
 * Phase 1 (Spin-up): Sets flywheel RPM and hood angle from the lookup table based on robot position. Continuously
 * updates as the robot moves. Waits for flywheel to reach setpoint.
 *
 * <p>
 * Phase 2 (Feeding): Once the flywheel is at speed, hood is at position, and turret is aimed, runs the conveyor and
 * agitates the intake actuator to feed game pieces.
 *
 * <p>
 * This command never finishes on its own - it runs until interrupted (button released). The turret is NOT required by
 * this command; its default aim command continues independently.
 */
public class ShootCommand extends Command {

    private final Flywheel flywheel;
    private final Hood hood;
    private final Conveyor conveyor;
    private final Turret turret;
    private final TurretAimingCalculator aimingCalculator;
    private final CommandSwerveDrivetrain drivetrain;
    private final Supplier<SwerveRequest> driveRequestSupplier;
    private final BooleanSupplier driverActive;
    private final IntakeActuator intakeActuator;
    private final SwerveRequest.RobotCentric wiggleRequest = new SwerveRequest.RobotCentric();

    private static final double WIGGLE_AMPLITUDE_RAD = Math.toRadians(5.0);
    private static final double WIGGLE_ROTATIONAL_RATE = Math.toRadians(40.0); // rad/s

    private boolean isFeeding;
    private boolean wasDriving;
    private boolean waitingForExtend;
    private double wiggleCenterRad;
    private boolean wiggleDirectionPositive;

    /** Teleop constructor - includes drivetrain brake control and intake actuator. */
    public ShootCommand(
            Flywheel flywheel,
            Hood hood,
            Conveyor conveyor,
            Turret turret,
            TurretAimingCalculator aimingCalculator,
            CommandSwerveDrivetrain drivetrain,
            Supplier<SwerveRequest> driveRequestSupplier,
            BooleanSupplier driverActive,
            IntakeActuator intakeActuator) {
        this.flywheel = flywheel;
        this.hood = hood;
        this.conveyor = conveyor;
        this.turret = turret;
        this.aimingCalculator = aimingCalculator;
        this.drivetrain = drivetrain;
        this.driveRequestSupplier = driveRequestSupplier;
        this.driverActive = driverActive;
        this.intakeActuator = intakeActuator;

        addRequirements(flywheel, hood, conveyor, drivetrain, intakeActuator);
    }

    /** Auto constructor - no drivetrain brake control. */
    public ShootCommand(
            Flywheel flywheel,
            Hood hood,
            Conveyor conveyor,
            Turret turret,
            TurretAimingCalculator aimingCalculator) {
        this.flywheel = flywheel;
        this.hood = hood;
        this.conveyor = conveyor;
        this.turret = turret;
        this.aimingCalculator = aimingCalculator;
        this.drivetrain = null;
        this.driveRequestSupplier = null;
        this.driverActive = null;
        this.intakeActuator = null;

        addRequirements(flywheel, hood, conveyor);
    }

    @Override
    public void initialize() {
        isFeeding = false;
        wasDriving = false;
        waitingForExtend = false;
        // if (intakeActuator != null) {
        // intakeActuator.resetAgitate();
        // }
    }

    @Override
    public void execute() {
        // Get flywheel RPM and hood angle — SOTM-compensated in teleop, standard in auto
        double targetRPM;
        double targetHoodAngle;
        if (drivetrain != null) {
            var sotmResult = aimingCalculator.calculateSOTM();
            targetRPM = sotmResult.flywheelRPM();
            targetHoodAngle = sotmResult.hoodAngleDegrees();
        } else {
            targetRPM = aimingCalculator.getFlywheelRPM();
            targetHoodAngle = aimingCalculator.getHoodAngle();
        }

        flywheel.setVelocity(targetRPM);
        hood.setPosition(targetHoodAngle);

        // Check if all conditions are met to begin feeding
        boolean flywheelReady = flywheel.atSetpoint();
        boolean hoodReady = hood.atPosition();
        boolean turretReady = true; // turret.isAtPosition();

        if (flywheelReady && hoodReady && turretReady) {
            isFeeding = true;
            conveyor.runFeed();
        } else {
            if (isFeeding) {
                conveyor.stop();
            }
            isFeeding = false;
        }

        // Wiggle spin or driver control (teleop only)
        if (drivetrain != null) {
            boolean driving = driverActive.getAsBoolean();
            if (driving) {
                drivetrain.setControl(driveRequestSupplier.get());
                // Keep intake extended while driving
                // if (intakeActuator != null) {
                // intakeActuator.driveExtend();
                // }
                wasDriving = true;
                waitingForExtend = false;
            } else {
                // Just stopped driving - capture heading as wiggle center
                if (wasDriving) {
                    wiggleCenterRad = drivetrain.getState().Pose.getRotation().getRadians();
                    wiggleDirectionPositive = true;
                    wasDriving = false;

                    // if (intakeActuator != null) {
                    // waitingForExtend = true;
                    // intakeActuator.driveExtend();
                    // intakeActuator.resetAgitate();
                    // }
                }

                // Wiggle: rotate ±5 degrees from center
                double currentRad = drivetrain.getState().Pose.getRotation().getRadians();
                double error = currentRad - wiggleCenterRad;
                if (wiggleDirectionPositive && error >= WIGGLE_AMPLITUDE_RAD) {
                    wiggleDirectionPositive = false;
                } else if (!wiggleDirectionPositive && error <= -WIGGLE_AMPLITUDE_RAD) {
                    wiggleDirectionPositive = true;
                }
                double rate = wiggleDirectionPositive ? WIGGLE_ROTATIONAL_RATE : -WIGGLE_ROTATIONAL_RATE;
                drivetrain.setControl(wiggleRequest.withRotationalRate(rate));

                // Intake actuator: extend first, then agitate
                // if (intakeActuator != null) {
                // if (waitingForExtend) {
                // intakeActuator.driveExtend();
                // if (intakeActuator.isExtended()) {
                // waitingForExtend = false;
                // }
                // } else {
                // intakeActuator.runAgitate();
                // }
                // }
            }
        }

        // Telemetry
        // SmartDashboard.putBoolean("Shoot/Feeding", isFeeding);
        // SmartDashboard.putBoolean("Shoot/FlywheelReady", flywheelReady);
        // SmartDashboard.putBoolean("Shoot/HoodReady", hoodReady);
        // SmartDashboard.putBoolean("Shoot/TurretReady", turretReady);
        // SmartDashboard.putNumber("Shoot/TargetRPM", targetRPM);
        // SmartDashboard.putNumber("Shoot/TargetHoodAngle", targetHoodAngle);
    }

    @Override
    public void end(boolean interrupted) {
        flywheel.setVelocity(0);
        conveyor.stop();
        // if (intakeActuator != null) {
        // intakeActuator.driveExtend();
        // }
        isFeeding = false;
    }

    @Override
    public boolean isFinished() {
        return false;
    }

}
