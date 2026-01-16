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
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants.QuestNavConstants;
import gg.questnav.questnav.QuestNav;
import gg.questnav.questnav.PoseFrame;

public class QuestNavSubsystem extends SubsystemBase {

    private final QuestNav questNav;
    private final CommandSwerveDrivetrain drivetrain;

    // Transform from robot center to Quest headset
    private final Transform3d robotToQuest;

    // Standard deviations for vision measurements
    private final Matrix<N3, N1> visionStdDevs;

    // State tracking
    private boolean isConnected = false;
    private Pose2d latestRobotPose = new Pose2d();

    public QuestNavSubsystem(CommandSwerveDrivetrain drivetrain) {
        this.drivetrain = drivetrain;
        this.questNav = new QuestNav();

        // Build transform from constants
        this.robotToQuest = new Transform3d(
            new Translation3d(
                QuestNavConstants.QUEST_OFFSET_X_METERS,
                QuestNavConstants.QUEST_OFFSET_Y_METERS,
                QuestNavConstants.QUEST_OFFSET_Z_METERS
            ),
            new Rotation3d(
                Math.toRadians(QuestNavConstants.QUEST_ROLL_OFFSET_DEGREES),
                Math.toRadians(QuestNavConstants.QUEST_PITCH_OFFSET_DEGREES),
                Math.toRadians(QuestNavConstants.QUEST_YAW_OFFSET_DEGREES)
            )
        );

        // Build standard deviation matrix
        this.visionStdDevs = VecBuilder.fill(
            QuestNavConstants.STD_DEV_X,
            QuestNavConstants.STD_DEV_Y,
            QuestNavConstants.STD_DEV_THETA
        );
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

                // Transform Quest pose to robot pose
                Pose3d questPose = frame.questPose3d();
                Pose3d robotPose3d = questPose.transformBy(robotToQuest.inverse());
                Pose2d robotPose = robotPose3d.toPose2d();

                latestRobotPose = robotPose;

                // Feed to drivetrain's Kalman filter
                drivetrain.addVisionMeasurement(
                    robotPose,
                    frame.dataTimestamp(),
                    visionStdDevs
                );
            } else {
                isConnected = false;
            }
        }

        // Telemetry
        SmartDashboard.putBoolean("QuestNav/Connected", isConnected);
        SmartDashboard.putNumber("QuestNav/Pose X", latestRobotPose.getX());
        SmartDashboard.putNumber("QuestNav/Pose Y", latestRobotPose.getY());
        SmartDashboard.putNumber("QuestNav/Pose Rotation", latestRobotPose.getRotation().getDegrees());
    }

    /**
     * Reset the QuestNav pose to match a known robot position.
     * Call this when resetting odometry (e.g., at start of auto).
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
}
