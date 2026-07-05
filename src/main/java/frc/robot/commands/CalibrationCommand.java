package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.FlywheelConstants;
import frc.robot.Constants.HoodConstants;
import frc.robot.Constants.ShooterModelConstants;
import frc.robot.subsystems.Flywheel;
import frc.robot.subsystems.Hood;
import frc.robot.util.ShotMap;
import frc.robot.util.TunableNumber;
import frc.robot.util.TurretAimingCalculator;

/**
 * Test-mode shooter calibration — replaces the old webapp + 578-cell grid
 * session with a handful of distance-keyed points.
 *
 * <p>
 * Procedure (see SHOOTER_TUNING.md):
 * <ol>
 * <li>Enable Test mode. The turret keeps auto-aiming; this command drives the
 * hood and flywheel from the {@code Calibration/HoodAngle} and
 * {@code Calibration/FlywheelRPM} dashboard values.</li>
 * <li>Park at a distance, dial hood/RPM until shots land, feed balls with the
 * driver right trigger.</li>
 * <li>Press record (driver A or the dashboard button). The point is keyed to
 * the current distance-to-target and saved on the roboRIO immediately.</li>
 * <li>Repeat at 4-6 distances across the range. Done.</li>
 * </ol>
 *
 * The hub map is recorded inside the scoring zone, the lob map outside — the
 * active map follows the robot's position automatically.
 */
public class CalibrationCommand extends Command {

    private final Hood hood;
    private final Flywheel flywheel;
    private final TurretAimingCalculator calculator;

    private final TunableNumber hoodInput = new TunableNumber(
            "Calibration/HoodAngle", ShooterModelConstants.EMPTY_MAP_HOOD_DEGREES);
    private final TunableNumber rpmInput = new TunableNumber(
            "Calibration/FlywheelRPM", ShooterModelConstants.EMPTY_MAP_FLYWHEEL_RPM);

    public CalibrationCommand(Hood hood, Flywheel flywheel, TurretAimingCalculator calculator) {
        this.hood = hood;
        this.flywheel = flywheel;
        this.calculator = calculator;
        addRequirements(hood, flywheel);
    }

    @Override
    public void initialize() {
        SmartDashboard.putString("Calibration/LastRecorded", "");
    }

    @Override
    public void execute() {
        double hoodSetpoint = MathUtil.clamp(hoodInput.get(),
                HoodConstants.MIN_POSITION_DEGREES, HoodConstants.MAX_POSITION_DEGREES);
        double rpmSetpoint = MathUtil.clamp(rpmInput.get(), 0.0, FlywheelConstants.MAX_RPM);
        hood.setPosition(hoodSetpoint);
        flywheel.setVelocity(rpmSetpoint);

        var target = calculator.getCalibrationTarget();
        ShotMap map = activeMap(target);
        SmartDashboard.putNumber("Calibration/DistanceMeters", target.distanceMeters());
        SmartDashboard.putString("Calibration/ActiveMap", map.getName().toUpperCase());
        SmartDashboard.putNumber("Calibration/PointCount", map.getPointCount());
        // What the current map would command here — when these already match
        // where shots land, this distance doesn't need a new point
        SmartDashboard.putNumber("Calibration/MapHoodAtDistance", map.getHoodDegrees(target.distanceMeters()));
        SmartDashboard.putNumber("Calibration/MapRPMAtDistance", map.getFlywheelRPM(target.distanceMeters()));
    }

    @Override
    public void end(boolean interrupted) {
        flywheel.setVelocity(0);
        hood.setPosition(0);
    }

    /**
     * Records the current dashboard hood/RPM at the current distance into the
     * active map and persists it. Bind to a button.
     */
    public void recordPoint() {
        if (!isScheduled()) {
            SmartDashboard.putString("Calibration/LastRecorded", "ignored - not in calibration mode");
            return;
        }
        var target = calculator.getCalibrationTarget();
        ShotMap map = activeMap(target);
        double hoodValue = MathUtil.clamp(hoodInput.get(),
                HoodConstants.MIN_POSITION_DEGREES, HoodConstants.MAX_POSITION_DEGREES);
        double rpmValue = MathUtil.clamp(rpmInput.get(), 0.0, FlywheelConstants.MAX_RPM);
        map.addPoint(target.distanceMeters(), hoodValue, rpmValue);
        String summary = String.format("%s: %.2f m -> hood %.1f deg, %.0f RPM (%d points)",
                map.getName(), target.distanceMeters(), hoodValue, rpmValue, map.getPointCount());
        SmartDashboard.putString("Calibration/LastRecorded", summary);
        System.out.println("[Calibration] recorded " + summary);
    }

    /** Removes the most recently recorded point from the active map. */
    public void undoLastPoint() {
        if (!isScheduled()) {
            SmartDashboard.putString("Calibration/LastRecorded", "ignored - not in calibration mode");
            return;
        }
        var target = calculator.getCalibrationTarget();
        ShotMap map = activeMap(target);
        boolean removed = map.removeLastPoint();
        String summary = removed
                ? String.format("%s: undid last point (%d points)", map.getName(), map.getPointCount())
                : String.format("%s: nothing to undo", map.getName());
        SmartDashboard.putString("Calibration/LastRecorded", summary);
        System.out.println("[Calibration] " + summary);
    }

    /** Dumps both maps to the console for pasting into Constants defaults. */
    public void printMaps() {
        System.out.println("=== ShotMap[hub] ===\n" + calculator.getHubMap().toCsv());
        System.out.println("=== ShotMap[lob] ===\n" + calculator.getLobMap().toCsv());
    }

    private ShotMap activeMap(TurretAimingCalculator.CalibrationTarget target) {
        return target.isHubShot() ? calculator.getHubMap() : calculator.getLobMap();
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
