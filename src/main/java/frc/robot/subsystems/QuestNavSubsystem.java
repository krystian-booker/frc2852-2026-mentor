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
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants.QuestNavConstants;
import gg.questnav.questnav.QuestNav;
import gg.questnav.questnav.PoseFrame;

public class QuestNavSubsystem extends SubsystemBase {

    private final QuestNav questNav;
    private final CommandSwerveDrivetrain drivetrain;
    private final Vision vision;

    // Transforms between robot center and Quest headset
    private final Transform3d robotToQuest;
    private final Transform3d questToRobot;

    // Standard deviations for vision measurements
    private final Matrix<N3, N1> visionStdDevs;

    // State tracking
    private boolean isConnected = false;
    private boolean isSeeded = false;
    private Pose2d latestRobotPose = new Pose2d();

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
        this.questToRobot = robotToQuest.inverse();

        // Build standard deviation matrix
        this.visionStdDevs = VecBuilder.fill(
                QuestNavConstants.STD_DEV_X,
                QuestNavConstants.STD_DEV_Y,
                QuestNavConstants.STD_DEV_THETA);

    }

    @Override
    public void periodic() {
        // Required: must call commandPeriodic() every loop
        questNav.commandPeriodic();

        isConnected = questNav.isConnected();

        for (PoseFrame frame : questNav.getAllUnreadPoseFrames()) {
            if (!frame.isTracking()) {
                // Tracking loss (USB drop, camera occlusion) invalidates the
                // Quest's pose stream. Stop feeding the estimator; the seeding
                // command re-seeds automatically at the next 2-tag vision view.
                if (isSeeded) {
                    DriverStation.reportWarning(
                            "QuestNav lost tracking - waiting for vision reseed", false);
                }
                isSeeded = false;
                continue;
            }

            // Transform Quest pose to robot pose
            Pose3d robotPose3d = frame.questPose3d().transformBy(questToRobot);
            latestRobotPose = robotPose3d.toPose2d();

            // Only feed to drivetrain after seeded from vision
            if (isSeeded) {
                drivetrain.addVisionMeasurement(
                        latestRobotPose,
                        frame.dataTimestamp(),
                        visionStdDevs);
            }
        }

        // Telemetry
        SmartDashboard.putBoolean("QuestNav/Connected", isConnected);
        SmartDashboard.putBoolean("QuestNav/Seeded", isSeeded);
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
        questNav.setPose(questPose);
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
     * Attempts to immediately seed QuestNav pose from vision. Requires the
     * latest vision estimate to be a fresh multi-tag solve — a single-tag
     * fallback (even with two tags in view across cameras) can be ambiguous
     * and must never become the hard-reset pose.
     *
     * @return true if seeding was successful
     */
    public boolean seedPoseFromVision() {
        if (vision.getLatestEstimateTagCount() >= 2 && vision.hasRecentValidPose(0.5)) {
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
