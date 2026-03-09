package frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.networktables.BooleanPublisher;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.DoubleSubscriber;
import edu.wpi.first.networktables.IntegerPublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StringPublisher;
import edu.wpi.first.wpilibj2.command.Command;

import frc.robot.Constants.CalibrationConstants;
import frc.robot.Constants.FlywheelConstants;
import frc.robot.Constants.HoodConstants;
import frc.robot.subsystems.Conveyor;
import frc.robot.subsystems.Flywheel;
import frc.robot.subsystems.Hood;
import frc.robot.util.TurretAimingCalculator;

/**
 * Calibration command for tuning turret lookup tables.
 * Acts as a thin client - only handles hardware I/O.
 * The webapp is the single source of truth for all calibration data.
 */
public class TurretCalibrationCommand extends Command {

    private final Hood hood;
    private final Flywheel flywheel;
    private final Conveyor conveyor;
    private final Supplier<Pose2d> poseSupplier;
    private final TurretAimingCalculator aimingCalculator;

    // NetworkTables
    private final NetworkTable table;

    // Publishers (Robot -> Webapp) - Essential state only
    private final BooleanPublisher enabledPub;
    private final DoublePublisher positionXPub;
    private final DoublePublisher positionYPub;
    private final DoublePublisher distancePub;
    private final DoublePublisher actualHoodAnglePub;
    private final DoublePublisher actualFlywheelRPMPub;
    private final BooleanPublisher hoodAtPositionPub;
    private final BooleanPublisher flywheelAtSetpointPub;
    private final IntegerPublisher gridRowPub;
    private final IntegerPublisher gridColPub;
    private final DoublePublisher nextTargetXPub;
    private final DoublePublisher nextTargetYPub;
    private final StringPublisher statusPub;
    private final StringPublisher calibrationInfoPub;

    // Subscribers (Webapp -> Robot) - Setpoint inputs only
    private final DoubleSubscriber inputHoodAngleSub;
    private final DoubleSubscriber inputFlywheelRPMSub;

    // Retained publishers (must be stored to prevent GC from unpublishing)
    @SuppressWarnings("unused")
    private final DoublePublisher[] retainedPublishers;

    /**
     * Creates a new TurretCalibrationCommand.
     *
     * @param hood              The hood subsystem
     * @param flywheel          The flywheel subsystem
     * @param conveyor          The conveyor subsystem
     * @param poseSupplier      Supplier for the robot's current pose
     * @param aimingCalculator  The turret aiming calculator for distance calculation
     */
    public TurretCalibrationCommand(Hood hood, Flywheel flywheel, Conveyor conveyor,
            Supplier<Pose2d> poseSupplier, TurretAimingCalculator aimingCalculator) {
        this.hood = hood;
        this.flywheel = flywheel;
        this.conveyor = conveyor;
        this.poseSupplier = poseSupplier;
        this.aimingCalculator = aimingCalculator;

        // Initialize NetworkTables
        table = NetworkTableInstance.getDefault().getTable("TurretCalibration");

        // Publishers
        enabledPub = table.getBooleanTopic("Enabled").publish();
        positionXPub = table.getDoubleTopic("Position/X").publish();
        positionYPub = table.getDoubleTopic("Position/Y").publish();
        distancePub = table.getDoubleTopic("Distance").publish();
        actualHoodAnglePub = table.getDoubleTopic("Actual/HoodAngle").publish();
        actualFlywheelRPMPub = table.getDoubleTopic("Actual/FlywheelRPM").publish();
        hoodAtPositionPub = table.getBooleanTopic("Actual/HoodAtPosition").publish();
        flywheelAtSetpointPub = table.getBooleanTopic("Actual/FlywheelAtSetpoint").publish();
        gridRowPub = table.getIntegerTopic("Grid/CurrentRow").publish();
        gridColPub = table.getIntegerTopic("Grid/CurrentCol").publish();
        nextTargetXPub = table.getDoubleTopic("Grid/NextTargetX").publish();
        nextTargetYPub = table.getDoubleTopic("Grid/NextTargetY").publish();
        statusPub = table.getStringTopic("Status").publish();
        calibrationInfoPub = table.getStringTopic("Info").publish();

        // Subscribers
        inputHoodAngleSub = table.getDoubleTopic("Input/HoodAngle")
                .subscribe(CalibrationConstants.DEFAULT_HOOD_ANGLE);
        inputFlywheelRPMSub = table.getDoubleTopic("Input/FlywheelRPM")
                .subscribe(CalibrationConstants.DEFAULT_FLYWHEEL_RPM);

        // Publish constants for webapp to read
        // Publishers must be stored to prevent GC from unpublishing the topics
        DoublePublisher minRPMPub = table.getDoubleTopic("Constants/MinFlywheelRPM").publish();
        minRPMPub.set(FlywheelConstants.MIN_RPM);
        DoublePublisher maxRPMPub = table.getDoubleTopic("Constants/MaxFlywheelRPM").publish();
        maxRPMPub.set(FlywheelConstants.MAX_RPM);
        DoublePublisher minHoodPub = table.getDoubleTopic("Constants/MinHoodAngle").publish();
        minHoodPub.set(HoodConstants.MIN_POSITION_DEGREES);
        DoublePublisher maxHoodPub = table.getDoubleTopic("Constants/MaxHoodAngle").publish();
        maxHoodPub.set(HoodConstants.MAX_POSITION_DEGREES);
        retainedPublishers = new DoublePublisher[] {
                minRPMPub, maxRPMPub, minHoodPub, maxHoodPub
        };
        table.getIntegerTopic("Grid/Rows").publish().set(CalibrationConstants.GRID_ROWS);
        table.getIntegerTopic("Grid/Cols").publish().set(CalibrationConstants.GRID_COLS);

        addRequirements(hood, flywheel, conveyor);
    }

    @Override
    public void initialize() {
        enabledPub.set(true);
        statusPub.set("Calibration Mode Active");
        calibrationInfoPub.set("Webapp is source of truth for calibration data");
    }

    @Override
    public void execute() {
        // Get current pose and calculate derived values
        Pose2d pose = poseSupplier.get();
        if (pose == null) {
            statusPub.set("Error: Pose unavailable - check drivetrain");
            positionXPub.set(0);
            positionYPub.set(0);
            distancePub.set(0);
            gridRowPub.set(-1);
            gridColPub.set(-1);
            return;
        }

        double robotX = pose.getX();
        double robotY = pose.getY();

        TurretAimingCalculator.AimingResult aimingResult = aimingCalculator.calculate();
        double distance = aimingResult.distanceMeters();

        // Calculate current grid position
        int gridCol = calculateGridCol(robotX);
        int gridRow = calculateGridRow(robotY);

        // Read user inputs from webapp and apply to hardware
        double inputHoodAngle = inputHoodAngleSub.get();
        double inputFlywheelRPM = inputFlywheelRPMSub.get();
        hood.setPosition(inputHoodAngle);
        flywheel.setVelocity(inputFlywheelRPM);
        conveyor.runFeed();

        // Publish robot state to webapp
        positionXPub.set(robotX);
        positionYPub.set(robotY);
        distancePub.set(distance);
        actualHoodAnglePub.set(hood.getCurrentPositionDegrees());
        actualFlywheelRPMPub.set(flywheel.getCurrentVelocityRPM());
        hoodAtPositionPub.set(hood.atPosition());
        flywheelAtSetpointPub.set(flywheel.atSetpoint());
        gridRowPub.set(gridRow);
        gridColPub.set(gridCol);

        // Calculate and publish next target position
        double[] nextTarget = calculateNextTarget(gridRow, gridCol);
        nextTargetXPub.set(nextTarget[0]);
        nextTargetYPub.set(nextTarget[1]);
    }

    @Override
    public void end(boolean interrupted) {
        enabledPub.set(false);
        flywheel.setVelocity(0);
        hood.setNeutral();
        conveyor.stop();

        if (interrupted) {
            statusPub.set("Calibration Interrupted");
        } else {
            statusPub.set("Calibration Ended");
        }
    }

    @Override
    public boolean isFinished() {
        return false; // Runs until interrupted
    }

    /**
     * Calculates the grid column index from X position.
     */
    private int calculateGridCol(double x) {
        if (x < CalibrationConstants.CALIBRATION_START_X) {
            return 0;
        }
        if (x > CalibrationConstants.CALIBRATION_END_X) {
            return CalibrationConstants.GRID_COLS - 1;
        }
        return (int) ((x - CalibrationConstants.CALIBRATION_START_X)
                / CalibrationConstants.GRID_CELL_SIZE_METERS);
    }

    /**
     * Calculates the grid row index from Y position.
     */
    private int calculateGridRow(double y) {
        if (y < CalibrationConstants.CALIBRATION_START_Y) {
            return 0;
        }
        if (y > CalibrationConstants.CALIBRATION_END_Y) {
            return CalibrationConstants.GRID_ROWS - 1;
        }
        return (int) ((y - CalibrationConstants.CALIBRATION_START_Y)
                / CalibrationConstants.GRID_CELL_SIZE_METERS);
    }

    /**
     * Calculates the next target position for the calibration grid.
     * Moves right along rows, then down to next row.
     *
     * @return Array of [x, y] for next target
     */
    private double[] calculateNextTarget(int currentRow, int currentCol) {
        int nextCol = currentCol + 1;
        int nextRow = currentRow;

        // If at end of row, move to next row
        if (nextCol >= CalibrationConstants.GRID_COLS) {
            nextCol = 0;
            nextRow = currentRow + 1;
        }

        // Clamp row to grid bounds
        if (nextRow >= CalibrationConstants.GRID_ROWS) {
            nextRow = CalibrationConstants.GRID_ROWS - 1;
            nextCol = CalibrationConstants.GRID_COLS - 1;
        }

        double nextX = CalibrationConstants.CALIBRATION_START_X
                + nextCol * CalibrationConstants.GRID_CELL_SIZE_METERS;
        double nextY = CalibrationConstants.CALIBRATION_START_Y
                + nextRow * CalibrationConstants.GRID_CELL_SIZE_METERS;

        return new double[] { nextX, nextY };
    }
}
