package frc.robot.commands;

import java.util.function.BooleanSupplier;
import java.util.function.Supplier;

import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.networktables.BooleanPublisher;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.DoubleSubscriber;
import edu.wpi.first.networktables.IntegerPublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StringPublisher;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;

import frc.robot.Constants.CalibrationConstants;
import frc.robot.Constants.FlywheelConstants;
import frc.robot.Constants.HoodConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.Flywheel;
import frc.robot.subsystems.Hood;
import frc.robot.util.DiagnosticLogger;
import frc.robot.util.TurretAimingCalculator;

/**
 * Calibration command for tuning turret lookup tables. Acts as a thin client -
 * only handles hardware I/O. The webapp is
 * the single source of truth for all calibration data.
 */
public class TurretCalibrationCommand extends Command {

    private final Hood hood;
    private final Flywheel flywheel;
    private final Indexer indexer;
    private final Supplier<Pose2d> poseSupplier;
    private final TurretAimingCalculator aimingCalculator;
    private final CommandSwerveDrivetrain drivetrain;
    private final Supplier<SwerveRequest> driveRequestSupplier;
    private final BooleanSupplier driverActive;
    private final BooleanSupplier feedActive;
    private final SwerveRequest.SwerveDriveBrake brakeRequest = new SwerveRequest.SwerveDriveBrake();
    private final DiagnosticLogger logger;

    // NetworkTables
    private final NetworkTable table;

    // Publishers (Robot -> Webapp) - Essential state only
    private final BooleanPublisher enabledPub;
    private final DoublePublisher positionXPub;
    private final DoublePublisher positionYPub;
    private final DoublePublisher distancePub;
    private final DoublePublisher actualHoodAnglePub;
    private final DoublePublisher actualFlywheelRPMPub;
    private final DoublePublisher observedInputHoodAnglePub;
    private final DoublePublisher observedInputFlywheelRPMPub;
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
    private double lastLoggedInputHoodAngle = Double.NaN;
    private double lastLoggedInputFlywheelRPM = Double.NaN;
    private boolean lastLoggedFeedActive = false;

    /**
     * Creates a new TurretCalibrationCommand.
     *
     * @param hood                 The hood subsystem
     * @param flywheel             The flywheel subsystem
     * @param indexer              The indexer subsystem
     * @param poseSupplier         Supplier for the robot's current pose
     * @param aimingCalculator     The turret aiming calculator for distance
     *                             calculation
     * @param drivetrain           The swerve drivetrain (locked in X-brake when
     *                             driver is idle)
     * @param driveRequestSupplier Supplier for the driver's swerve request
     * @param driverActive         Whether the driver is actively controlling the
     *                             drivetrain
     * @param feedActive           Whether the driver is actively requesting feed
     */
    public TurretCalibrationCommand(Hood hood, Flywheel flywheel, Indexer indexer,
            Supplier<Pose2d> poseSupplier, TurretAimingCalculator aimingCalculator,
            CommandSwerveDrivetrain drivetrain,
            Supplier<SwerveRequest> driveRequestSupplier,
            BooleanSupplier driverActive,
            BooleanSupplier feedActive) {
        this.hood = hood;
        this.flywheel = flywheel;
        this.indexer = indexer;
        this.poseSupplier = poseSupplier;
        this.aimingCalculator = aimingCalculator;
        this.drivetrain = drivetrain;
        this.driveRequestSupplier = driveRequestSupplier;
        this.driverActive = driverActive;
        this.feedActive = feedActive;
        this.logger = new DiagnosticLogger("turret_calibration", new String[] {
                "timestamp",
                "pose_available",
                "driver_active",
                "feed_active",
                "robot_x",
                "robot_y",
                "distance_m",
                "grid_row",
                "grid_col",
                "input_hood_deg",
                "hood_target_deg",
                "actual_hood_deg",
                "hood_at_position",
                "input_flywheel_rpm",
                "flywheel_target_rpm",
                "actual_flywheel_rpm",
                "flywheel_at_setpoint"
        });

        // Initialize NetworkTables
        table = NetworkTableInstance.getDefault().getTable("TurretCalibration");

        // Publishers
        enabledPub = table.getBooleanTopic("Enabled").publish();
        positionXPub = table.getDoubleTopic("Position/X").publish();
        positionYPub = table.getDoubleTopic("Position/Y").publish();
        distancePub = table.getDoubleTopic("Distance").publish();
        actualHoodAnglePub = table.getDoubleTopic("Actual/HoodAngle").publish();
        actualFlywheelRPMPub = table.getDoubleTopic("Actual/FlywheelRPM").publish();
        observedInputHoodAnglePub = table.getDoubleTopic("ObservedInput/HoodAngle").publish();
        observedInputFlywheelRPMPub = table.getDoubleTopic("ObservedInput/FlywheelRPM").publish();
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

        addRequirements(drivetrain);
    }

    @Override
    public void initialize() {
        enabledPub.set(true);
        statusPub.set("Calibration Mode Active");
        calibrationInfoPub.set("Webapp controls hood/flywheel continuously in test mode; right trigger feeds");
        SmartDashboard.putBoolean("Calibration/CommandActive", true);
        SmartDashboard.putString("Calibration/LogFile", "");
        logger.open();
        SmartDashboard.putString("Calibration/LogFile", logger.getCurrentFilePath());
        DriverStation.reportWarning("Turret calibration logging started: " + logger.getCurrentFilePath(), false);
        lastLoggedInputHoodAngle = Double.NaN;
        lastLoggedInputFlywheelRPM = Double.NaN;
        lastLoggedFeedActive = false;
    }

    @Override
    public void execute() {
        double inputHoodAngle = inputHoodAngleSub.get();
        double inputFlywheelRPM = inputFlywheelRPMSub.get();
        boolean feedRequested = feedActive.getAsBoolean();
        boolean driverDriving = driverActive.getAsBoolean();

        // Get current pose and calculate derived values
        Pose2d pose = poseSupplier.get();
        if (pose == null) {
            statusPub.set("Error: Pose unavailable - check drivetrain");
            positionXPub.set(0);
            positionYPub.set(0);
            distancePub.set(0);
            gridRowPub.set(-1);
            gridColPub.set(-1);
            SmartDashboard.putString("Calibration/Status", "Pose unavailable");
            SmartDashboard.putNumber("Calibration/InputHoodAngle", inputHoodAngle);
            SmartDashboard.putNumber("Calibration/InputFlywheelRPM", inputFlywheelRPM);
            observedInputHoodAnglePub.set(inputHoodAngle);
            observedInputFlywheelRPMPub.set(inputFlywheelRPM);
            SmartDashboard.putBoolean("Calibration/FeedRequested", feedRequested);
            SmartDashboard.putBoolean("Calibration/DriverActive", driverDriving);
            logStateChange(inputHoodAngle, inputFlywheelRPM, feedRequested);
            if (logger.isOpen()) {
                logger.logRow(
                        Timer.getFPGATimestamp(),
                        0.0,
                        driverDriving ? 1.0 : 0.0,
                        feedRequested ? 1.0 : 0.0,
                        Double.NaN,
                        Double.NaN,
                        Double.NaN,
                        -1.0,
                        -1.0,
                        inputHoodAngle,
                        hood.getTargetPositionDegrees(),
                        hood.getCurrentPositionDegrees(),
                        hood.atPosition() ? 1.0 : 0.0,
                        inputFlywheelRPM,
                        flywheel.getTargetVelocityRPM(),
                        flywheel.getCurrentVelocityRPM(),
                        flywheel.atSetpoint() ? 1.0 : 0.0);
            }
            return;
        }

        double robotX = pose.getX();
        double robotY = pose.getY();

        TurretAimingCalculator.AimingResult aimingResult = aimingCalculator.calculate();
        double distance = aimingResult.distanceMeters();

        // Calculate current grid position
        int gridCol = calculateGridCol(robotX);
        int gridRow = calculateGridRow(robotY);

        double actualHoodAngle = hood.getCurrentPositionDegrees();
        double actualFlywheelRPM = flywheel.getCurrentVelocityRPM();
        boolean hoodAtPosition = hood.atPosition();
        boolean flywheelAtSetpoint = flywheel.atSetpoint();

        // Read user inputs from webapp for logging and dashboard visibility.
        SmartDashboard.putNumber("Calibration/InputHoodAngle", inputHoodAngle);
        SmartDashboard.putNumber("Calibration/InputFlywheelRPM", inputFlywheelRPM);
        observedInputHoodAnglePub.set(inputHoodAngle);
        observedInputFlywheelRPMPub.set(inputFlywheelRPM);
        SmartDashboard.putNumber("Calibration/TargetHoodAngle", hood.getTargetPositionDegrees());
        SmartDashboard.putNumber("Calibration/TargetFlywheelRPM", flywheel.getTargetVelocityRPM());
        SmartDashboard.putNumber("Calibration/ActualHoodAngle", actualHoodAngle);
        SmartDashboard.putNumber("Calibration/ActualFlywheelRPM", actualFlywheelRPM);
        SmartDashboard.putBoolean("Calibration/HoodAtPosition", hoodAtPosition);
        SmartDashboard.putBoolean("Calibration/FlywheelAtSetpoint", flywheelAtSetpoint);
        SmartDashboard.putBoolean("Calibration/FeedRequested", feedRequested);
        SmartDashboard.putBoolean("Calibration/DriverActive", driverDriving);
        SmartDashboard.putString("Calibration/Status", feedRequested ? "Calibration Active - Feeding"
                : "Calibration Active - Waiting");
        logStateChange(inputHoodAngle, inputFlywheelRPM, feedRequested);

        // Lock wheels in X-brake when driver is not actively driving
        if (driverDriving) {
            drivetrain.setControl(driveRequestSupplier.get());
        } else {
            drivetrain.setControl(brakeRequest);
        }

        // Publish robot state to webapp
        positionXPub.set(robotX);
        positionYPub.set(robotY);
        distancePub.set(distance);
        actualHoodAnglePub.set(actualHoodAngle);
        actualFlywheelRPMPub.set(actualFlywheelRPM);
        hoodAtPositionPub.set(hoodAtPosition);
        flywheelAtSetpointPub.set(flywheelAtSetpoint);
        gridRowPub.set(gridRow);
        gridColPub.set(gridCol);

        // Calculate and publish next target position
        double[] nextTarget = calculateNextTarget(gridRow, gridCol);
        nextTargetXPub.set(nextTarget[0]);
        nextTargetYPub.set(nextTarget[1]);

        if (logger.isOpen()) {
            logger.logRow(
                    Timer.getFPGATimestamp(),
                    1.0,
                    driverDriving ? 1.0 : 0.0,
                    feedRequested ? 1.0 : 0.0,
                    robotX,
                    robotY,
                    distance,
                    gridRow,
                    gridCol,
                    inputHoodAngle,
                    hood.getTargetPositionDegrees(),
                    actualHoodAngle,
                    hoodAtPosition ? 1.0 : 0.0,
                    inputFlywheelRPM,
                    flywheel.getTargetVelocityRPM(),
                    actualFlywheelRPM,
                    flywheelAtSetpoint ? 1.0 : 0.0);
        }
    }

    @Override
    public void end(boolean interrupted) {
        enabledPub.set(false);
        indexer.stop();
        SmartDashboard.putBoolean("Calibration/CommandActive", false);
        SmartDashboard.putString("Calibration/Status", interrupted ? "Calibration Interrupted" : "Calibration Ended");
        logger.close();

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
     * Calculates the next target position for the calibration grid. Moves right
     * along rows, then down to next row.
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

    private void logStateChange(double inputHoodAngle, double inputFlywheelRPM, boolean feedRequested) {
        boolean hoodChanged = Double.isNaN(lastLoggedInputHoodAngle)
                || Math.abs(inputHoodAngle - lastLoggedInputHoodAngle) > 1e-6;
        boolean flywheelChanged = Double.isNaN(lastLoggedInputFlywheelRPM)
                || Math.abs(inputFlywheelRPM - lastLoggedInputFlywheelRPM) > 1e-6;
        boolean feedChanged = feedRequested != lastLoggedFeedActive;

        if (hoodChanged || flywheelChanged || feedChanged) {
            System.out.printf(
                    "TurretCalibration: hoodInput=%.2f deg, flywheelInput=%.1f rpm, feed=%s%n",
                    inputHoodAngle,
                    inputFlywheelRPM,
                    feedRequested ? "ON" : "OFF");
            lastLoggedInputHoodAngle = inputHoodAngle;
            lastLoggedInputFlywheelRPM = inputFlywheelRPM;
            lastLoggedFeedActive = feedRequested;
        }
    }
}
