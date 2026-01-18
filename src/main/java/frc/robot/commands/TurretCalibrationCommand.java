package frc.robot.commands;

import java.util.Optional;
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
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;

import frc.robot.Constants.CalibrationConstants;
import frc.robot.Constants.HoodConstants;
import frc.robot.Constants.TurretAimingConstants;
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

    // Validation publishers
    private final BooleanPublisher readyToSavePub;
    private final StringPublisher validationWarningPub;
    private final BooleanPublisher distanceValidPub;
    private final BooleanPublisher hoodAngleValidPub;
    private final BooleanPublisher flywheelRPMValidPub;
    private final BooleanPublisher allianceMismatchPub;
    private final BooleanPublisher duplicatePointWarningPub;

    // Validation constants
    private static final double MIN_FLYWHEEL_RPM = 1000.0;
    private static final double MAX_FLYWHEEL_RPM = 6000.0;

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

        // Validation publishers
        readyToSavePub = table.getBooleanTopic("Validation/ReadyToSave").publish();
        validationWarningPub = table.getStringTopic("Validation/Warning").publish();
        distanceValidPub = table.getBooleanTopic("Validation/DistanceValid").publish();
        hoodAngleValidPub = table.getBooleanTopic("Validation/HoodAngleValid").publish();
        flywheelRPMValidPub = table.getBooleanTopic("Validation/FlywheelRPMValid").publish();
        allianceMismatchPub = table.getBooleanTopic("Validation/AllianceMismatch").publish();
        duplicatePointWarningPub = table.getBooleanTopic("Validation/DuplicatePoint").publish();

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
        if (pose == null) {
            statusPub.set("Error: Pose unavailable - check drivetrain");
            return;
        }
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
        actualFlywheelRPMPub.set(flywheel.getCurrentVelocityRPM());
        hoodAtPositionPub.set(hood.atPosition());
        flywheelAtSetpointPub.set(flywheel.atSetpoint());
        gridRowPub.set(gridRow);
        gridColPub.set(gridCol);

        // Calculate and publish next target position
        double[] nextTarget = calculateNextTarget(gridRow, gridCol);
        nextTargetXPub.set(nextTarget[0]);
        nextTargetYPub.set(nextTarget[1]);

        // Perform validation checks
        boolean hoodAtPosition = hood.atPosition();
        boolean flywheelAtSetpoint = flywheel.atSetpoint();
        boolean distanceValid = isDistanceValid(distance);
        boolean hoodAngleValid = isHoodAngleValid(inputHoodAngle);
        boolean flywheelRPMValid = isFlywheelRPMValid(inputFlywheelRPM);
        boolean allianceMismatch = isAllianceMismatch(selectedAlliance);
        boolean hasDuplicatePoint = dataRecorder.hasPointAt(gridRow, gridCol);

        // Publish validation state
        distanceValidPub.set(distanceValid);
        hoodAngleValidPub.set(hoodAngleValid);
        flywheelRPMValidPub.set(flywheelRPMValid);
        allianceMismatchPub.set(allianceMismatch);
        duplicatePointWarningPub.set(hasDuplicatePoint);

        // Build validation warning message
        StringBuilder warnings = new StringBuilder();
        if (!hoodAtPosition) {
            warnings.append("Hood not at setpoint. ");
        }
        if (!flywheelAtSetpoint) {
            warnings.append("Flywheel not at setpoint. ");
        }
        if (!distanceValid) {
            warnings.append(String.format("Distance %.1fm outside valid range (%.1f-%.1fm). ",
                    distance, TurretAimingConstants.MIN_SHOOTING_DISTANCE_METERS,
                    TurretAimingConstants.MAX_SHOOTING_DISTANCE_METERS));
        }
        if (!hoodAngleValid) {
            warnings.append(String.format("Hood angle %.1f° outside valid range (%.0f-%.0f°). ",
                    inputHoodAngle, HoodConstants.MIN_POSITION_DEGREES, HoodConstants.MAX_POSITION_DEGREES));
        }
        if (!flywheelRPMValid) {
            warnings.append(String.format("Flywheel RPM %.0f outside valid range (%.0f-%.0f). ",
                    inputFlywheelRPM, MIN_FLYWHEEL_RPM, MAX_FLYWHEEL_RPM));
        }
        if (allianceMismatch) {
            warnings.append("Selected alliance differs from DriverStation. ");
        }
        if (hasDuplicatePoint) {
            warnings.append("Point exists at this grid cell. ");
        }

        // Determine if ready to save (setpoints reached)
        boolean readyToSave = hoodAtPosition && flywheelAtSetpoint;
        readyToSavePub.set(readyToSave);
        validationWarningPub.set(warnings.toString());

        // Handle Save Point button (edge-triggered on rising edge)
        boolean currentSavePoint = savePointSub.get();
        if (currentSavePoint && !lastSavePointValue) {
            trySaveCurrentPoint(robotX, robotY, distance, inputHoodAngle, inputFlywheelRPM,
                    gridRow, gridCol, selectedAlliance, readyToSave, hasDuplicatePoint);
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
     * Validates distance is within shooting range.
     */
    private boolean isDistanceValid(double distance) {
        return distance >= TurretAimingConstants.MIN_SHOOTING_DISTANCE_METERS
                && distance <= TurretAimingConstants.MAX_SHOOTING_DISTANCE_METERS;
    }

    /**
     * Validates hood angle is within mechanical limits.
     */
    private boolean isHoodAngleValid(double hoodAngle) {
        return hoodAngle >= HoodConstants.MIN_POSITION_DEGREES
                && hoodAngle <= HoodConstants.MAX_POSITION_DEGREES;
    }

    /**
     * Validates flywheel RPM is within reasonable range.
     */
    private boolean isFlywheelRPMValid(double rpm) {
        return rpm >= MIN_FLYWHEEL_RPM && rpm <= MAX_FLYWHEEL_RPM;
    }

    /**
     * Checks if selected alliance differs from DriverStation alliance.
     */
    private boolean isAllianceMismatch(String selectedAlliance) {
        Optional<Alliance> dsAlliance = DriverStation.getAlliance();
        if (dsAlliance.isEmpty()) {
            return false; // No mismatch if DriverStation alliance unknown
        }
        String dsAllianceStr = dsAlliance.get() == Alliance.Blue ? "Blue" : "Red";
        return !selectedAlliance.equals(dsAllianceStr);
    }

    /**
     * Attempts to save the current calibration point with validation.
     */
    private void trySaveCurrentPoint(double robotX, double robotY, double distance,
            double hoodAngle, double flywheelRPM, int gridRow, int gridCol, String alliance,
            boolean readyToSave, boolean hasDuplicatePoint) {

        // Check if setpoints are reached
        if (!readyToSave) {
            statusPub.set("Cannot save: Hood/Flywheel not at setpoint!");
            return;
        }

        // Handle duplicate point - update existing instead of adding new
        if (hasDuplicatePoint) {
            dataRecorder.updatePointAt(gridRow, gridCol, robotX, robotY, distance,
                    hoodAngle, flywheelRPM, alliance);
            statusPub.set(String.format("Point Updated at [%d,%d]!", gridRow, gridCol));
        } else {
            dataRecorder.addPoint(robotX, robotY, distance, hoodAngle, flywheelRPM,
                    gridRow, gridCol, alliance);
            int count = dataRecorder.getPointCount();
            int total = dataRecorder.getTotalGridCells();
            statusPub.set(String.format("Point Saved! (%d/%d)", count, total));
        }

        pointsSavedPub.set(dataRecorder.getPointCount());

        // Auto-save after each point
        if (!dataRecorder.saveToFile()) {
            statusPub.set("Warning: Save failed! Check console.");
        }
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

}
