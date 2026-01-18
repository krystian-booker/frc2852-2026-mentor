package frc.robot.commands;

import java.util.Optional;
import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.networktables.BooleanPublisher;
import edu.wpi.first.networktables.BooleanSubscriber;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.DoubleSubscriber;
import edu.wpi.first.networktables.IntegerPublisher;
import edu.wpi.first.networktables.IntegerSubscriber;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StringArrayPublisher;
import edu.wpi.first.networktables.StringPublisher;
import edu.wpi.first.networktables.StringSubscriber;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;

import frc.robot.Constants.CalibrationConstants;
import frc.robot.Constants.FlywheelConstants;
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

    // Data freshness publisher
    private final DoublePublisher lastUpdateTimestampPub;

    // Subscribers (Elastic -> Robot)
    private final DoubleSubscriber inputHoodAngleSub;
    private final DoubleSubscriber inputFlywheelRPMSub;
    private final BooleanSubscriber savePointSub;
    private final BooleanSubscriber exportSub;
    private final BooleanSubscriber clearDataSub;
    private final BooleanSubscriber deletePointSub;
    private final StringSubscriber allianceSelectSub;

    // Targeted delete subscribers (for deleting specific points from webapp DataTable)
    private final IntegerSubscriber deleteTargetRowSub;
    private final IntegerSubscriber deleteTargetColSub;
    private final IntegerSubscriber deleteTargetSequenceSub;
    private final BooleanSubscriber deleteTargetPointSub;

    // State
    private boolean lastSavePointValue = false;
    private boolean lastExportValue = false;
    private boolean lastClearDataValue = false;
    private boolean lastDeletePointValue = false;
    private boolean lastDeleteTargetPointValue = false;
    private long lastProcessedDeleteSequence = 0; // For sequence validation

    // File save retry configuration
    private static final int MAX_SAVE_RETRIES = 3;
    private static final long SAVE_RETRY_DELAY_MS = 100;

    /**
     * Creates a new TurretCalibrationCommand.
     *
     * @param hood              The hood subsystem
     * @param flywheel          The flywheel subsystem
     * @param poseSupplier      Supplier for the robot's current pose
     * @param aimingCalculator  The turret aiming calculator for distance calculation
     * @param dataRecorder      Shared CalibrationDataRecorder instance for persistence
     */
    public TurretCalibrationCommand(Hood hood, Flywheel flywheel,
            Supplier<Pose2d> poseSupplier, TurretAimingCalculator aimingCalculator,
            CalibrationDataRecorder dataRecorder) {
        this.hood = hood;
        this.flywheel = flywheel;
        this.poseSupplier = poseSupplier;
        this.aimingCalculator = aimingCalculator;
        this.dataRecorder = dataRecorder;

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

        // Data freshness publisher
        lastUpdateTimestampPub = table.getSubTable("Data").getDoubleTopic("LastUpdateTimestamp").publish();

        // Subscribers with default values
        inputHoodAngleSub = table.getDoubleTopic("Input/HoodAngle")
                .subscribe(CalibrationConstants.DEFAULT_HOOD_ANGLE);
        inputFlywheelRPMSub = table.getDoubleTopic("Input/FlywheelRPM")
                .subscribe(CalibrationConstants.DEFAULT_FLYWHEEL_RPM);
        savePointSub = table.getBooleanTopic("SavePoint").subscribe(false);
        exportSub = table.getBooleanTopic("Export").subscribe(false);
        clearDataSub = table.getBooleanTopic("ClearData").subscribe(false);
        deletePointSub = table.getBooleanTopic("DeleteCurrentPoint").subscribe(false);
        allianceSelectSub = allianceTable.getStringTopic("active").subscribe("Blue");

        // Targeted delete subscribers (for webapp DataTable deletions)
        deleteTargetRowSub = table.getIntegerTopic("DeleteTarget/Row").subscribe(-1);
        deleteTargetColSub = table.getIntegerTopic("DeleteTarget/Col").subscribe(-1);
        deleteTargetSequenceSub = table.getIntegerTopic("DeleteTarget/Sequence").subscribe(0);
        deleteTargetPointSub = table.getBooleanTopic("DeleteTargetPoint").subscribe(false);

        // Set initial values for inputs so they appear in Elastic
        table.getDoubleTopic("Input/HoodAngle").publish()
                .set(CalibrationConstants.DEFAULT_HOOD_ANGLE);
        table.getDoubleTopic("Input/FlywheelRPM").publish()
                .set(CalibrationConstants.DEFAULT_FLYWHEEL_RPM);

        // Publish constants for webapp to read (one-time on startup)
        table.getDoubleTopic("Constants/MinFlywheelRPM").publish().set(FlywheelConstants.MIN_RPM);
        table.getDoubleTopic("Constants/MaxFlywheelRPM").publish().set(FlywheelConstants.MAX_RPM);
        table.getDoubleTopic("Constants/MinHoodAngle").publish().set(HoodConstants.MIN_POSITION_DEGREES);
        table.getDoubleTopic("Constants/MaxHoodAngle").publish().set(HoodConstants.MAX_POSITION_DEGREES);
        table.getIntegerTopic("Grid/Rows").publish().set(CalibrationConstants.GRID_ROWS);
        table.getIntegerTopic("Grid/Cols").publish().set(CalibrationConstants.GRID_COLS);

        addRequirements(hood, flywheel);
    }

    @Override
    public void initialize() {
        enabledPub.set(true);
        statusPub.set("Calibration Mode Active");
        totalPointsPub.set(dataRecorder.getTotalGridCells());
        pointsSavedPub.set(dataRecorder.getPointCount());
        calibrationInfoPub.set("Distance-based calibration works for both alliances!");

        // Publish current points to NetworkTables so webapp can sync on startup
        dataRecorder.publishPointsToNetworkTables();

        // Reset edge detection - initialize to true (defensive default)
        // This ensures the first edge detection requires a fresh button press,
        // avoiding race conditions if button state changed between command creation and initialization
        lastSavePointValue = true;
        lastExportValue = true;
        lastClearDataValue = true;
        lastDeletePointValue = true;
        lastDeleteTargetPointValue = true;
    }

    @Override
    public void execute() {
        // Get current pose and calculate derived values
        Pose2d pose = poseSupplier.get();
        if (pose == null) {
            statusPub.set("Error: Pose unavailable - check drivetrain");
            // Zero out position publishers to prevent stale data display
            positionXPub.set(0);
            positionYPub.set(0);
            distancePub.set(0);
            // Set grid position to invalid
            gridRowPub.set(-1);
            gridColPub.set(-1);
            // Ensure save is not possible without valid pose
            readyToSavePub.set(false);
            validationWarningPub.set("Pose unavailable - cannot save");
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

        // Read selected alliance (normalize: trim and capitalize first letter)
        String rawAlliance = allianceSelectSub.get();
        String selectedAlliance = normalizeAlliance(rawAlliance);
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
        boolean gridPositionValid = gridRow >= 0 && gridRow < CalibrationConstants.GRID_ROWS
                && gridCol >= 0 && gridCol < CalibrationConstants.GRID_COLS;
        boolean allianceValid = "Blue".equals(selectedAlliance) || "Red".equals(selectedAlliance);
        boolean positionClamped = isXOutsideBounds(robotX) || isYOutsideBounds(robotY);

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
                    inputFlywheelRPM, FlywheelConstants.MIN_RPM, FlywheelConstants.MAX_RPM));
        }
        if (allianceMismatch) {
            warnings.append("Selected alliance differs from DriverStation. ");
        }
        if (hasDuplicatePoint) {
            warnings.append("Point exists at this grid cell. ");
        }
        if (!gridPositionValid) {
            warnings.append(String.format("Grid position [%d,%d] outside valid bounds. ", gridRow, gridCol));
        }
        if (!allianceValid) {
            warnings.append(String.format("Invalid alliance '%s' (must be Blue or Red). ", selectedAlliance));
        }
        if (positionClamped) {
            warnings.append(String.format("Robot at (%.1f, %.1f) is outside calibration area - position clamped. ",
                    robotX, robotY));
        }

        // Determine if ready to save (setpoints reached, values within valid ranges, and position/alliance valid)
        boolean readyToSave = hoodAtPosition && flywheelAtSetpoint
                && distanceValid && hoodAngleValid && flywheelRPMValid
                && gridPositionValid && allianceValid;
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

        // Handle Delete Point button (edge-triggered)
        boolean currentDeletePoint = deletePointSub.get();
        if (currentDeletePoint && !lastDeletePointValue) {
            handleDeletePoint(gridRow, gridCol);
        }
        lastDeletePointValue = currentDeletePoint;

        // Handle Delete Target Point (from webapp DataTable - deletes specific row/col, not current position)
        // Uses sequence number validation to prevent race conditions - only processes if sequence > lastProcessed
        boolean currentDeleteTargetPoint = deleteTargetPointSub.get();
        if (currentDeleteTargetPoint && !lastDeleteTargetPointValue) {
            int targetRow = (int) deleteTargetRowSub.get();
            int targetCol = (int) deleteTargetColSub.get();
            long sequence = deleteTargetSequenceSub.get();

            // Only process if sequence is newer than last processed
            if (sequence > lastProcessedDeleteSequence) {
                handleDeleteTargetPoint(targetRow, targetCol);
                lastProcessedDeleteSequence = sequence;
            } else {
                // Stale delete request - log but don't process
                System.out.println("Ignoring stale delete request (seq=" + sequence + ", lastProcessed=" + lastProcessedDeleteSequence + ")");
            }
        }
        lastDeleteTargetPointValue = currentDeleteTargetPoint;

        // Publish timestamp for data freshness indicator (seconds since epoch)
        lastUpdateTimestampPub.set(System.currentTimeMillis() / 1000.0);
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
     * Checks if the X position is outside the calibration bounds (being clamped).
     */
    private boolean isXOutsideBounds(double x) {
        return x < CalibrationConstants.CALIBRATION_START_X
                || x > CalibrationConstants.CALIBRATION_END_X;
    }

    /**
     * Checks if the Y position is outside the calibration bounds (being clamped).
     */
    private boolean isYOutsideBounds(double y) {
        return y < CalibrationConstants.CALIBRATION_START_Y
                || y > CalibrationConstants.CALIBRATION_END_Y;
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
        return rpm >= FlywheelConstants.MIN_RPM && rpm <= FlywheelConstants.MAX_RPM;
    }

    /**
     * Normalizes alliance string: trims whitespace and capitalizes properly.
     * Handles case-insensitivity ("blue" -> "Blue", "RED" -> "Red").
     */
    private String normalizeAlliance(String alliance) {
        if (alliance == null) {
            return "Blue"; // Default
        }
        String trimmed = alliance.trim().toLowerCase();
        if (trimmed.equals("red")) {
            return "Red";
        } else if (trimmed.equals("blue")) {
            return "Blue";
        }
        return "Blue"; // Default for invalid input
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

        // Check for valid distance (protects against null/unavailable pose)
        if (distance <= 0) {
            statusPub.set("Cannot save: Invalid distance - check pose data!");
            return;
        }

        // Check if setpoints are reached
        if (!readyToSave) {
            statusPub.set("Cannot save: Hood/Flywheel not at setpoint!");
            return;
        }

        // Handle duplicate point - update existing instead of adding new
        try {
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
        } catch (IllegalArgumentException e) {
            String errorMsg = "Save failed: " + e.getMessage();
            statusPub.set(errorMsg);
            DriverStation.reportError("Calibration save error: " + e.getMessage(), false);
            return;
        }

        pointsSavedPub.set(dataRecorder.getPointCount());

        // Auto-save after each point with retry logic
        saveWithRetry("Point save");
    }

    /**
     * Attempts to save calibration data with retry logic.
     * @param context Description of the save context for error messages
     */
    private void saveWithRetry(String context) {
        for (int attempt = 1; attempt <= MAX_SAVE_RETRIES; attempt++) {
            if (dataRecorder.saveToFile()) {
                return; // Success
            }

            // Log retry attempt
            String retryMsg = String.format("%s: Save attempt %d/%d failed, retrying...",
                    context, attempt, MAX_SAVE_RETRIES);
            System.err.println(retryMsg);
            statusPub.set(retryMsg);

            // Wait before retry (except on last attempt)
            if (attempt < MAX_SAVE_RETRIES) {
                try {
                    Thread.sleep(SAVE_RETRY_DELAY_MS);
                } catch (InterruptedException e) {
                    Thread.currentThread().interrupt();
                    break;
                }
            }
        }

        // All retries failed - CRITICAL warning
        String criticalMsg = "CRITICAL: " + context + " - all " + MAX_SAVE_RETRIES +
                " save attempts failed! Data may be lost on reboot!";
        System.err.println(criticalMsg);
        statusPub.set(criticalMsg);
        DriverStation.reportError(criticalMsg, false);
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
     * Handles the delete point command for the current grid cell.
     */
    private void handleDeletePoint(int gridRow, int gridCol) {
        boolean removed = dataRecorder.removePointAt(gridRow, gridCol);
        if (removed) {
            pointsSavedPub.set(dataRecorder.getPointCount());
            statusPub.set(String.format("Point deleted at [%d,%d]", gridRow, gridCol));
            // Auto-save after deletion with retry logic
            saveWithRetry("Delete save");
        } else {
            statusPub.set(String.format("No point to delete at [%d,%d]", gridRow, gridCol));
        }
    }

    /**
     * Handles the delete target point command from the webapp DataTable.
     * Deletes a specific point at the given row/col, not necessarily the current position.
     */
    private void handleDeleteTargetPoint(int targetRow, int targetCol) {
        if (targetRow < 0 || targetCol < 0) {
            statusPub.set("Delete failed: Invalid target position");
            return;
        }

        boolean removed = dataRecorder.removePointAt(targetRow, targetCol);
        if (removed) {
            pointsSavedPub.set(dataRecorder.getPointCount());
            statusPub.set(String.format("Point deleted at [%d,%d]", targetRow, targetCol));
            // Auto-save after deletion with retry logic
            saveWithRetry("Target delete save");
        } else {
            statusPub.set(String.format("No point to delete at [%d,%d]", targetRow, targetCol));
        }
    }

}
