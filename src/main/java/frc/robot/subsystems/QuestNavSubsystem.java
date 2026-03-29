package frc.robot.subsystems;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.util.datalog.DoubleLogEntry;
import edu.wpi.first.util.datalog.BooleanLogEntry;

import frc.robot.Constants.QuestNavConstants;
import gg.questnav.questnav.QuestNav;
import gg.questnav.questnav.PoseFrame;

public class QuestNavSubsystem extends SubsystemBase {

    private final QuestNav questNav;
    private final CommandSwerveDrivetrain drivetrain;
    private final Vision vision;

    // Transform from robot center to Quest headset
    private final Transform3d robotToQuest;

    // Standard deviations for vision measurements
    private final Matrix<N3, N1> visionStdDevs;

    // State tracking
    private boolean isConnected = false;
    private boolean isSeeded = false;
    private Pose2d latestRobotPose = new Pose2d();

    // Reseed timing instrumentation
    private double lastSeedTimestamp = 0.0;
    private boolean waitingForFirstFrameAfterSeed = false;
    private final DoubleLogEntry seedCallDurationLog;
    private final DoubleLogEntry convergenceTimeLog;
    private final BooleanLogEntry seedEventLog;

    public QuestNavSubsystem(CommandSwerveDrivetrain drivetrain, Vision vision) {
        this.drivetrain = drivetrain;
        this.vision = vision;
        this.questNav = new QuestNav();

        // Build transform from constants
        this.robotToQuest = new Transform3d(
                new Translation3d(
                        QuestNavConstants.QUEST_OFFSET_X_METERS,
                        QuestNavConstants.QUEST_OFFSET_Y_METERS,
                        QuestNavConstants.QUEST_OFFSET_Z_METERS),
                new Rotation3d(
                        Math.toRadians(QuestNavConstants.QUEST_ROLL_OFFSET_DEGREES),
                        Math.toRadians(QuestNavConstants.QUEST_PITCH_OFFSET_DEGREES),
                        Math.toRadians(QuestNavConstants.QUEST_YAW_OFFSET_DEGREES)));

        // Build standard deviation matrix
        this.visionStdDevs = VecBuilder.fill(
                QuestNavConstants.STD_DEV_X,
                QuestNavConstants.STD_DEV_Y,
                QuestNavConstants.STD_DEV_THETA);

        // Reseed timing logs
        var log = DataLogManager.getLog();
        seedCallDurationLog = new DoubleLogEntry(log, "/QuestNav/SeedCallDurationMs");
        convergenceTimeLog = new DoubleLogEntry(log, "/QuestNav/ConvergenceTimeMs");
        seedEventLog = new BooleanLogEntry(log, "/QuestNav/SeedEvent");
    }

    @Override
    public void periodic() {
        // Required: must call commandPeriodic() every loop
        questNav.commandPeriodic();

        // Process all unread pose frames
        PoseFrame[] frames = questNav.getAllUnreadPoseFrames();

        for (PoseFrame frame : frames) {
            if (frame.isTracking()) {
                isConnected = true;

                // Measure convergence: time from setPose() call to first valid tracking frame
                if (waitingForFirstFrameAfterSeed) {
                    double convergenceMs = (Timer.getFPGATimestamp() - lastSeedTimestamp) * 1000.0;
                    convergenceTimeLog.append(convergenceMs);
                    SmartDashboard.putNumber("QuestNav/LastConvergenceMs", convergenceMs);
                    System.out.println("[QuestNav] First tracking frame after reseed: " + convergenceMs + " ms");
                    waitingForFirstFrameAfterSeed = false;
                }

                // Transform Quest pose to robot pose
                Pose3d questPose = frame.questPose3d();
                Pose3d robotPose3d = questPose.transformBy(robotToQuest.inverse());
                Pose2d robotPose = robotPose3d.toPose2d();

                latestRobotPose = robotPose;

                // Track divergence between QuestNav and drivetrain pose estimate
                double divergenceMeters = robotPose.getTranslation().getDistance(
                        drivetrain.getState().Pose.getTranslation());
                SmartDashboard.putNumber("QuestNav/DivergenceMeters", divergenceMeters);

                // Only feed to drivetrain after seeded from vision
                if (isSeeded) {
                    drivetrain.addVisionMeasurement(
                            robotPose,
                            frame.dataTimestamp(),
                            visionStdDevs);
                }
            } else {
                isConnected = false;
            }
        }

        // Telemetry
        SmartDashboard.putBoolean("QuestNav/Connected", isConnected);
        SmartDashboard.putBoolean("QuestNav/Seeded", isSeeded);
        SmartDashboard.putBoolean("QuestNav/WaitingForConvergence", waitingForFirstFrameAfterSeed);
        SmartDashboard.putNumber("QuestNav/Battery",
                this.questNav.getBatteryPercent().isPresent() ? this.questNav.getBatteryPercent().getAsInt() : 0);
    }

    /**
     * Reset the QuestNav pose to match a known robot position. Call this when resetting odometry (e.g., at start of
     * auto).
     */
    public void resetPose(Pose2d robotPose) {
        Pose3d robotPose3d = new Pose3d(robotPose);
        Pose3d questPose = robotPose3d.transformBy(robotToQuest);

        double startNanos = System.nanoTime();
        questNav.setPose(questPose);
        double elapsedMs = (System.nanoTime() - startNanos) / 1_000_000.0;

        // Log the setPose() call duration
        seedCallDurationLog.append(elapsedMs);
        seedEventLog.append(true);
        SmartDashboard.putNumber("QuestNav/LastSeedCallMs", elapsedMs);
        System.out.println("[QuestNav] setPose() call took " + elapsedMs + " ms");

        // Start tracking convergence time
        lastSeedTimestamp = Timer.getFPGATimestamp();
        waitingForFirstFrameAfterSeed = true;
    }

    /** Returns true if QuestNav is actively tracking. */
    public boolean isTracking() {
        return isConnected;
    }

    /** Returns the latest robot pose from QuestNav. */
    public Pose2d getLatestPose() {
        return latestRobotPose;
    }

    /**
     * Attempts to immediately seed QuestNav pose from vision. Requires at least 2 visible tags for accurate seeding.
     * 
     * @return true if seeding was successful (valid vision pose with 2+ tags)
     */
    public boolean seedPoseFromVision() {
        if (vision.getVisibleTagCount() >= 2 && vision.hasRecentValidPose(0.5)) {
            var visionPose = vision.getLatestPose2d();
            if (visionPose.isPresent()) {
                resetPose(visionPose.get());
                drivetrain.resetPose(visionPose.get());
                isSeeded = true;
                return true;
            }
        }
        return false;
    }

    /** Returns true if QuestNav has been seeded from vision. */
    public boolean isSeeded() {
        return isSeeded;
    }

    /** Clears the seeded state, stopping QuestNav from feeding the drivetrain until re-seeded. */
    public void clearSeeded() {
        isSeeded = false;
    }

    /**
     * Creates a command that waits for a valid vision pose and then seeds QuestNav.
     * 
     * @param timeout Maximum time to wait for a valid pose in seconds
     * @return Command that seeds QuestNav when vision pose is available
     */
    public Command seedFromVisionCommand(double timeout) {
        Timer timer = new Timer();
        return Commands.sequence(
                Commands.runOnce(timer::restart),
                Commands.waitUntil(() -> vision.hasRecentValidPose(0.5) || timer.hasElapsed(timeout)),
                Commands.runOnce(() -> {
                    if (vision.hasRecentValidPose(0.5)) {
                        seedPoseFromVision();
                    }
                }));
    }

    /**
     * Creates an instant command that attempts to seed QuestNav from vision. Does nothing if no valid vision pose is
     * available.
     * 
     * @return Command that attempts immediate vision seeding
     */
    public Command trySeedFromVision() {
        return Commands.runOnce(this::seedPoseFromVision);
    }
}
