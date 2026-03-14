package frc.robot.commands;

import java.util.function.BooleanSupplier;
import java.util.function.Supplier;

import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;

import frc.robot.Constants.IntakeActuatorConstants;
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
    private final IntakeActuator intakeActuator;
    private final Turret turret;
    private final TurretAimingCalculator aimingCalculator;
    private final CommandSwerveDrivetrain drivetrain;
    private final Supplier<SwerveRequest> driveRequestSupplier;
    private final BooleanSupplier driverActive;
    private final SwerveRequest.SwerveDriveBrake brakeRequest = new SwerveRequest.SwerveDriveBrake();

    private boolean isFeeding;
    private boolean agitateRetract;
    private long agitateTimer;

    /** Teleop constructor - includes drivetrain brake control. */
    public ShootCommand(
            Flywheel flywheel,
            Hood hood,
            Conveyor conveyor,
            IntakeActuator intakeActuator,
            Turret turret,
            TurretAimingCalculator aimingCalculator,
            CommandSwerveDrivetrain drivetrain,
            Supplier<SwerveRequest> driveRequestSupplier,
            BooleanSupplier driverActive) {
        this.flywheel = flywheel;
        this.hood = hood;
        this.conveyor = conveyor;
        this.intakeActuator = intakeActuator;
        this.turret = turret;
        this.aimingCalculator = aimingCalculator;
        this.drivetrain = drivetrain;
        this.driveRequestSupplier = driveRequestSupplier;
        this.driverActive = driverActive;

        addRequirements(flywheel, hood, conveyor, intakeActuator, drivetrain);
    }

    /** Auto constructor - no drivetrain brake control. */
    public ShootCommand(
            Flywheel flywheel,
            Hood hood,
            Conveyor conveyor,
            IntakeActuator intakeActuator,
            Turret turret,
            TurretAimingCalculator aimingCalculator) {
        this.flywheel = flywheel;
        this.hood = hood;
        this.conveyor = conveyor;
        this.intakeActuator = intakeActuator;
        this.turret = turret;
        this.aimingCalculator = aimingCalculator;
        this.drivetrain = null;
        this.driveRequestSupplier = null;
        this.driverActive = null;

        addRequirements(flywheel, hood, conveyor, intakeActuator);
    }

    @Override
    public void initialize() {
        isFeeding = false;
        agitateRetract = true;
        agitateTimer = System.currentTimeMillis();
    }

    @Override
    public void execute() {
        // Continuously update flywheel RPM and hood angle from LUT
        double targetRPM = aimingCalculator.getFlywheelRPM();
        double targetHoodAngle = aimingCalculator.getHoodAngle();

        flywheel.setVelocity(targetRPM);
        hood.setPosition(targetHoodAngle);

        // Check if all conditions are met to begin feeding
        boolean flywheelReady = flywheel.atSetpoint();
        boolean hoodReady = hood.atPosition();
        boolean turretReady = true; // turret.isAtPosition();

        if (flywheelReady && hoodReady && turretReady) {
            isFeeding = true;
            conveyor.runFeed();

            // Agitate intake actuator - alternate retract/extend on a timer
            double timeout = agitateRetract
                    ? IntakeActuatorConstants.AGITATE_RETRACT_SECONDS
                    : IntakeActuatorConstants.AGITATE_EXTEND_SECONDS;
            if (System.currentTimeMillis() - agitateTimer >= timeout * 1000) {
                agitateRetract = !agitateRetract;
                agitateTimer = System.currentTimeMillis();
            }
            if (agitateRetract) {
                intakeActuator.driveRetractOpenLoop();
            } else {
                intakeActuator.driveExtendOpenLoop();
            }
        } else {
            if (isFeeding) {
                conveyor.stop();
            }
            isFeeding = false;
            intakeActuator.driveExtend();
        }

        // Lock wheels in X-brake when driver is not actively driving (teleop only)
        if (drivetrain != null) {
            if (driverActive.getAsBoolean()) {
                drivetrain.setControl(driveRequestSupplier.get());
            } else {
                drivetrain.setControl(brakeRequest);
            }
        }

        // Telemetry
        SmartDashboard.putBoolean("Shoot/Feeding", isFeeding);
        SmartDashboard.putBoolean("Shoot/FlywheelReady", flywheelReady);
        SmartDashboard.putBoolean("Shoot/HoodReady", hoodReady);
        SmartDashboard.putBoolean("Shoot/TurretReady", turretReady);
        SmartDashboard.putNumber("Shoot/TargetRPM", targetRPM);
        SmartDashboard.putNumber("Shoot/TargetHoodAngle", targetHoodAngle);
    }

    @Override
    public void end(boolean interrupted) {
        intakeActuator.driveExtend();
        flywheel.setVelocity(0);
        conveyor.stop();
        isFeeding = false;
    }

    @Override
    public boolean isFinished() {
        return false;
    }

}
