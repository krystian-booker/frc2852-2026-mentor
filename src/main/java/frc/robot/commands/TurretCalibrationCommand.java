package frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.networktables.BooleanPublisher;
import edu.wpi.first.networktables.BooleanSubscriber;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.DoubleSubscriber;
import edu.wpi.first.networktables.IntegerPublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StringArrayPublisher;
import edu.wpi.first.networktables.StringPublisher;
import edu.wpi.first.networktables.StringSubscriber;
import edu.wpi.first.wpilibj2.command.Command;

import frc.robot.Constants.CalibrationConstants;
import frc.robot.subsystems.Flywheel;
import frc.robot.subsystems.Hood;
import frc.robot.util.CalibrationDataRecorder;
import frc.robot.util.TurretAimingCalculator;

/**
 * Main calibration command for tuning turret lookup tables.
 * Reads user inputs from NetworkTables (via Elastic dashboard),
 * applies values to hood and flywheel in real-time, and manages data recording.
 */
public class TurretCalibrationCommand extends Command {

    private final Hood hood;
    private final Flywheel flywheel;
    private final Supplier<Pose2d> poseSupplier;
    private final TurretAimingCalculator aimingCalculator;
    private final CalibrationDataRecorder dataRecorder;

    // NetworkTables
    private final NetworkTable table;

    // Publishers (Robot -> Elastic)
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
    private final IntegerPublisher pointsSavedPub;
    private final IntegerPublisher totalPointsPub;
    private final StringPublisher statusPub;
    private final StringPublisher allianceCurrentPub;
    private final StringPublisher calibrationInfoPub;
    private final StringArrayPublisher allianceOptionsPub;
    private final StringPublisher allianceSelectedPub;
    private final StringPublisher allianceActivePub;
    private final StringPublisher allianceDefaultPub;

    // Subscribers (Elastic -> Robot)
    private final DoubleSubscriber inputHoodAngleSub;
    private final DoubleSubscriber inputFlywheelRPMSub;
    private final BooleanSubscriber savePointSub;
    private final BooleanSubscriber exportSub;
    private final BooleanSubscriber clearDataSub;
    private final StringSubscriber allianceSelectSub;

    // State
    private boolean lastSavePointValue = false;
    private boolean lastExportValue = false;
    private boolean lastClearDataValue = false;

    /**
     * Creates a new TurretCalibrationCommand.
     *
     * @param hood              The hood subsystem
     * @param flywheel          The flywheel subsystem
     * @param poseSupplier      Supplier for the robot's current pose
     * @param aimingCalculator  The turret aiming calculator for distance calculation
     */
    public TurretCalibrationCommand(Hood hood, Flywheel flywheel,
            Supplier<Pose2d> poseSupplier, TurretAimingCalculator aimingCalculator) {
        this.hood = hood;
        this.flywheel = flywheel;
        this.poseSupplier = poseSupplier;
        this.aimingCalculator = aimingCalculator;
        this.dataRecorder = new CalibrationDataRecorder();

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
        pointsSavedPub = table.getIntegerTopic("Progress/PointsSaved").publish();
        totalPointsPub = table.getIntegerTopic("Progress/TotalPoints").publish();
        statusPub = table.getStringTopic("Status").publish();
        allianceCurrentPub = table.getStringTopic("Alliance/Current").publish();
        calibrationInfoPub = table.getStringTopic("Info").publish();

        // Alliance selection for ComboBox Chooser (SendableChooser-compatible format)
        NetworkTable allianceTable = table.getSubTable("Alliance").getSubTable("Select");
        allianceOptionsPub = allianceTable.getStringArrayTopic("options").publish();
        allianceSelectedPub = allianceTable.getStringTopic("selected").publish();
        allianceActivePub = allianceTable.getStringTopic("active").publish();
        allianceDefaultPub = allianceTable.getStringTopic("default").publish();

        // Publish alliance options for ComboBox
        String[] allianceOptions = new String[] { "Blue", "Red" };
        allianceOptionsPub.set(allianceOptions);
        allianceDefaultPub.set("Blue");
        allianceSelectedPub.set("Blue");
        allianceActivePub.set("Blue");

        // Subscribers with default values
        inputHoodAngleSub = table.getDoubleTopic("Input/HoodAngle")
                .subscribe(CalibrationConstants.DEFAULT_HOOD_ANGLE);
        inputFlywheelRPMSub = table.getDoubleTopic("Input/FlywheelRPM")
                .subscribe(CalibrationConstants.DEFAULT_FLYWHEEL_RPM);
        savePointSub = table.getBooleanTopic("SavePoint").subscribe(false);
        exportSub = table.getBooleanTopic("Export").subscribe(false);
        clearDataSub = table.getBooleanTopic("ClearData").subscribe(false);
        allianceSelectSub = allianceTable.getStringTopic("active").subscribe("Blue");

        // Set initial values for inputs so they appear in Elastic
        table.getDoubleTopic("Input/HoodAngle").publish()
                .set(CalibrationConstants.DEFAULT_HOOD_ANGLE);
        table.getDoubleTopic("Input/FlywheelRPM").publish()
                .set(CalibrationConstants.DEFAULT_FLYWHEEL_RPM);

        addRequirements(hood, flywheel);
    }

    @Override
    public void initialize() {
        enabledPub.set(true);
        statusPub.set("Calibration Mode Active");
        totalPointsPub.set(dataRecorder.getTotalGridCells());
        pointsSavedPub.set(dataRecorder.getPointCount());
        calibrationInfoPub.set("Distance-based calibration works for both alliances!");

        // Reset edge detection
        lastSavePointValue = savePointSub.get();
        lastExportValue = exportSub.get();
        lastClearDataValue = clearDataSub.get();
    }

    @Override
    public void execute() {
        // Get current pose and calculate derived values
        Pose2d pose = poseSupplier.get();
        double robotX = pose.getX();
        double robotY = pose.getY();

        TurretAimingCalculator.AimingResult aimingResult = aimingCalculator.calculate();
        double distance = aimingResult.distanceMeters();

        // Calculate current grid position
        int gridCol = calculateGridCol(robotX);
        int gridRow = calculateGridRow(robotY);

        // Read user inputs from Elastic
        double inputHoodAngle = inputHoodAngleSub.get();
        double inputFlywheelRPM = inputFlywheelRPMSub.get();

        // Read selected alliance
        String selectedAlliance = allianceSelectSub.get();
        allianceCurrentPub.set(selectedAlliance);

        // Apply inputs to subsystems in real-time
        hood.setPosition(inputHoodAngle);
        flywheel.setVelocity(inputFlywheelRPM);

        // Publish robot state to Elastic
        positionXPub.set(robotX);
        positionYPub.set(robotY);
        distancePub.set(distance);
        actualHoodAnglePub.set(hood.getCurrentPositionDegrees());
        actualFlywheelRPMPub.set(getCurrentFlywheelRPM());
        hoodAtPositionPub.set(hood.atPosition());
        flywheelAtSetpointPub.set(flywheel.atSetpoint());
        gridRowPub.set(gridRow);
        gridColPub.set(gridCol);

        // Calculate and publish next target position
        double[] nextTarget = calculateNextTarget(gridRow, gridCol);
        nextTargetXPub.set(nextTarget[0]);
        nextTargetYPub.set(nextTarget[1]);

        // Handle Save Point button (edge-triggered on rising edge)
        boolean currentSavePoint = savePointSub.get();
        if (currentSavePoint && !lastSavePointValue) {
            saveCurrentPoint(robotX, robotY, distance, inputHoodAngle, inputFlywheelRPM, gridRow, gridCol, selectedAlliance);
        }
        lastSavePointValue = currentSavePoint;

        // Handle Export button (edge-triggered)
        boolean currentExport = exportSub.get();
        if (currentExport && !lastExportValue) {
            handleExport();
        }
        lastExportValue = currentExport;

        // Handle Clear Data button (edge-triggered)
        boolean currentClearData = clearDataSub.get();
        if (currentClearData && !lastClearDataValue) {
            handleClearData();
        }
        lastClearDataValue = currentClearData;
    }

    @Override
    public void end(boolean interrupted) {
        enabledPub.set(false);
        flywheel.setVelocity(0);
        hood.setNeutral();

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

    /**
     * Saves the current calibration point.
     */
    private void saveCurrentPoint(double robotX, double robotY, double distance,
            double hoodAngle, double flywheelRPM, int gridRow, int gridCol, String alliance) {
        dataRecorder.addPoint(robotX, robotY, distance, hoodAngle, flywheelRPM, gridRow, gridCol, alliance);

        int count = dataRecorder.getPointCount();
        int total = dataRecorder.getTotalGridCells();
        pointsSavedPub.set(count);
        statusPub.set(String.format("Point Saved! (%d/%d)", count, total));

        // Auto-save after each point
        dataRecorder.saveToFile();
    }

    /**
     * Handles the export command.
     */
    private void handleExport() {
        boolean success = dataRecorder.exportFinal();
        if (success) {
            int count = dataRecorder.getPointCount();
            statusPub.set(String.format("Calibration Complete! %d points saved to %s",
                    count, dataRecorder.getFilePath()));
        } else {
            statusPub.set("Export Failed - Check console for errors");
        }
    }

    /**
     * Handles the clear data command.
     */
    private void handleClearData() {
        dataRecorder.clearData();
        pointsSavedPub.set(0);
        statusPub.set("All calibration data cleared");
    }

    /**
     * Gets the current flywheel velocity in RPM.
     * Uses SmartDashboard value since Flywheel doesn't expose getter directly.
     */
    private double getCurrentFlywheelRPM() {
        return edu.wpi.first.wpilibj.smartdashboard.SmartDashboard
                .getNumber("Flywheel/Velocity RPM", 0.0);
    }
}
