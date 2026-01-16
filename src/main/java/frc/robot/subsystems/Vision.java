package frc.robot.subsystems;

import static frc.robot.Constants.Vision.*;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import frc.robot.Constants.Vision.AprilTagTarget;

import java.util.HashSet;
import java.util.List;
import java.util.Optional;
import java.util.Set;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.simulation.PhotonCameraSim;
import org.photonvision.simulation.SimCameraProperties;
import org.photonvision.simulation.VisionSystemSim;
import org.photonvision.targeting.PhotonTrackedTarget;

import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathConstraints;

public class Vision extends SubsystemBase {
    private final PhotonCamera camera;
    private final PhotonPoseEstimator photonEstimator;
    private Matrix<N3, N1> curStdDevs;
    private final EstimateConsumer estConsumer;

    // State tracking for pose estimation
    private Pose2d latestEstimate = new Pose2d();
    private double latestEstimateTimestamp = 0.0;
    private final Set<Integer> visibleTagIds = new HashSet<>();

    // Feeding control - can be disabled after QuestNav seeding
    private boolean feedingEnabled = true;

    // Simulation
    private PhotonCameraSim cameraSim;
    private VisionSystemSim visionSim;

    /**
     * @param estConsumer Lambda that will accept a pose estimate and pass it to your desired {@link edu.wpi.first.math.estimator.SwerveDrivePoseEstimator}
     */
    public Vision(EstimateConsumer estConsumer) {
        this.estConsumer = estConsumer;
        camera = new PhotonCamera(kCameraName);
        photonEstimator = new PhotonPoseEstimator(kTagLayout, kRobotToCam);

        // ----- Simulation
        if (Robot.isSimulation()) {
            // Create the vision system simulation which handles cameras and targets on the field.
            visionSim = new VisionSystemSim("main");
            // Add all the AprilTags inside the tag layout as visible targets to this simulated field.
            visionSim.addAprilTags(kTagLayout);
            // Create simulated camera properties. These can be set to mimic your actual camera.
            var cameraProp = new SimCameraProperties();
            cameraProp.setCalibration(960, 720, Rotation2d.fromDegrees(90));
            cameraProp.setCalibError(0.35, 0.10);
            cameraProp.setFPS(15);
            cameraProp.setAvgLatencyMs(50);
            cameraProp.setLatencyStdDevMs(15);
            // Create a PhotonCameraSim which will update the linked PhotonCamera's values with visible
            // targets.
            cameraSim = new PhotonCameraSim(camera, cameraProp);
            // Add the simulated camera to view the targets on this simulated field.
            visionSim.addCamera(cameraSim, kRobotToCam);

            cameraSim.enableDrawWireframe(true);
        }
    }

    @Override
    public void periodic() {
        // Clear visible tags for this frame
        visibleTagIds.clear();

        Optional<EstimatedRobotPose> visionEst = Optional.empty();
        for (var result : camera.getAllUnreadResults()) {
            // Track visible tags
            for (var target : result.getTargets()) {
                visibleTagIds.add(target.getFiducialId());
            }

            visionEst = photonEstimator.estimateCoprocMultiTagPose(result);
            if (visionEst.isEmpty()) {
                visionEst = photonEstimator.estimateLowestAmbiguityPose(result);
            }
            updateEstimationStdDevs(visionEst, result.getTargets());

            if (Robot.isSimulation()) {
                visionEst.ifPresentOrElse(est -> getSimDebugField().getObject("VisionEstimation").setPose(est.estimatedPose.toPose2d()), () -> {
                    getSimDebugField().getObject("VisionEstimation").setPoses();
                });
            }

            visionEst.ifPresent(est -> {
                // Store latest estimate for pose access
                latestEstimate = est.estimatedPose.toPose2d();
                latestEstimateTimestamp = est.timestampSeconds;

                // Change our trust in the measurement based on the tags we can see
                var estStdDevs = getEstimationStdDevs();

                // Only feed drivetrain if feeding is enabled (disabled after QuestNav seeding)
                if (feedingEnabled) {
                    estConsumer.accept(est.estimatedPose.toPose2d(), est.timestampSeconds, estStdDevs);
                }
            });
        }

        // Telemetry
        SmartDashboard.putBoolean("Vision/HasValidPose", hasRecentValidPose(POSE_VALIDITY_TIMEOUT));
        SmartDashboard.putString("Vision/VisibleTags", visibleTagIds.toString());
        SmartDashboard.putNumber("Vision/PoseX", latestEstimate.getX());
        SmartDashboard.putNumber("Vision/PoseY", latestEstimate.getY());
        SmartDashboard.putNumber("Vision/PoseRotation", latestEstimate.getRotation().getDegrees());
        SmartDashboard.putBoolean("Vision/FeedingEnabled", feedingEnabled);
    }

    /**
     * Returns the latest estimated pose from vision.
     * @return Optional containing the pose if available, empty if no pose has been estimated
     */
    public Optional<Pose2d> getLatestPose2d() {
        if (latestEstimateTimestamp == 0.0) {
            return Optional.empty();
        }
        return Optional.of(latestEstimate);
    }

    /**
     * Checks if there is a recent valid pose estimate.
     * @param maxAgeSeconds Maximum age in seconds for the pose to be considered valid
     * @return true if a valid pose exists within the specified age
     */
    public boolean hasRecentValidPose(double maxAgeSeconds) {
        if (latestEstimateTimestamp == 0.0) {
            return false;
        }
        double age = Timer.getFPGATimestamp() - latestEstimateTimestamp;
        return age <= maxAgeSeconds;
    }

    /**
     * Returns the timestamp of the latest pose estimate.
     * @return Timestamp in seconds, or 0 if no estimate exists
     */
    public double getLatestEstimateTimestamp() {
        return latestEstimateTimestamp;
    }

    /**
     * Returns the set of currently visible AprilTag IDs.
     * @return Set of visible tag IDs
     */
    public Set<Integer> getVisibleTagIds() {
        return new HashSet<>(visibleTagIds);
    }

    /**
     * Checks if a specific AprilTag is currently visible.
     * @param tagId The tag ID to check
     * @return true if the tag is currently visible
     */
    public boolean isTagVisible(int tagId) {
        return visibleTagIds.contains(tagId);
    }

    /**
     * Returns the count of currently visible AprilTags.
     * @return Number of visible tags
     */
    public int getVisibleTagCount() {
        return visibleTagIds.size();
    }

    /**
     * Sets whether vision should feed pose estimates to the drivetrain.
     * @param enabled true to enable feeding, false to disable
     */
    public void setFeedingEnabled(boolean enabled) {
        this.feedingEnabled = enabled;
    }

    /**
     * Returns whether vision is currently feeding pose estimates to the drivetrain.
     * @return true if feeding is enabled
     */
    public boolean isFeedingEnabled() {
        return feedingEnabled;
    }

    /**
     * Returns the first visible configured AprilTagTarget, if any.
     * @return Optional containing a visible target, or empty if none visible
     */
    public Optional<AprilTagTarget> getVisibleTarget() {
        for (int tagId : visibleTagIds) {
            AprilTagTarget target = AprilTagTarget.fromTagId(tagId);
            if (target != null) {
                return Optional.of(target);
            }
        }
        return Optional.empty();
    }

    /**
     * Creates a command to pathfind to an AprilTag target position using PathPlanner.
     * @param drivetrain The swerve drivetrain
     * @param tagId The AprilTag ID to drive to
     * @return Command that pathfinds to the target, or does nothing if tag not configured
     */
    public Command pathfindToAprilTagTarget(CommandSwerveDrivetrain drivetrain, int tagId) {
        AprilTagTarget target = AprilTagTarget.fromTagId(tagId);
        if (target == null) {
            return Commands.none();
        }

        PathConstraints constraints = new PathConstraints(
            3.0, // max velocity m/s
            3.0, // max acceleration m/s^2
            2 * Math.PI, // max angular velocity rad/s
            4 * Math.PI  // max angular acceleration rad/s^2
        );

        return AutoBuilder.pathfindToPose(target.getTargetPose(), constraints);
    }

    /**
     * Creates a command to pathfind to any currently visible configured target.
     * @param drivetrain The swerve drivetrain
     * @return Command that pathfinds to a visible target, or does nothing if none visible
     */
    public Command pathfindToVisibleTarget(CommandSwerveDrivetrain drivetrain) {
        return Commands.defer(() -> {
            Optional<AprilTagTarget> target = getVisibleTarget();
            if (target.isPresent()) {
                return pathfindToAprilTagTarget(drivetrain, target.get().getTagId());
            }
            return Commands.none();
        }, Set.of(drivetrain));
    }

    /**
     * Creates a command to drive directly to an AprilTag target using simple PID control.
     * Use this for direct line driving when obstacles are not a concern.
     * @param drivetrain The swerve drivetrain
     * @param tagId The AprilTag ID to drive to
     * @return Command that drives to the target
     */
    public Command driveToAprilTagTarget(CommandSwerveDrivetrain drivetrain, int tagId) {
        AprilTagTarget target = AprilTagTarget.fromTagId(tagId);
        if (target == null) {
            return Commands.none();
        }

        PIDController xController = new PIDController(DRIVE_P, DRIVE_I, DRIVE_D);
        PIDController yController = new PIDController(DRIVE_P, DRIVE_I, DRIVE_D);
        PIDController rotController = new PIDController(ROTATION_P, ROTATION_I, ROTATION_D);
        rotController.enableContinuousInput(-Math.PI, Math.PI);

        xController.setTolerance(POSITION_TOLERANCE_METERS);
        yController.setTolerance(POSITION_TOLERANCE_METERS);
        rotController.setTolerance(ROTATION_TOLERANCE_RADIANS);

        Pose2d targetPose = target.getTargetPose();

        SwerveRequest.FieldCentric driveRequest = new SwerveRequest.FieldCentric();

        return Commands.run(() -> {
            Pose2d currentPose = drivetrain.getState().Pose;

            double xSpeed = xController.calculate(currentPose.getX(), targetPose.getX());
            double ySpeed = yController.calculate(currentPose.getY(), targetPose.getY());
            double rotSpeed = rotController.calculate(
                currentPose.getRotation().getRadians(),
                targetPose.getRotation().getRadians()
            );

            drivetrain.setControl(driveRequest
                .withVelocityX(xSpeed)
                .withVelocityY(ySpeed)
                .withRotationalRate(rotSpeed));
        }, drivetrain)
        .until(() -> xController.atSetpoint() && yController.atSetpoint() && rotController.atSetpoint())
        .finallyDo(() -> drivetrain.setControl(new SwerveRequest.SwerveDriveBrake()));
    }

    /**
     * Calculates new standard deviations. This algorithm is a heuristic that creates dynamic standard deviations based on number of tags, estimation strategy, and distance from the tags.
     *
     * @param estimatedPose The estimated pose to guess standard deviations for.
     * @param targets All targets in this camera frame
     */
    private void updateEstimationStdDevs(Optional<EstimatedRobotPose> estimatedPose, List<PhotonTrackedTarget> targets) {
        if (estimatedPose.isEmpty()) {
            // No pose input. Default to single-tag std devs
            curStdDevs = kSingleTagStdDevs;

        } else {
            // Pose present. Start running Heuristic
            var estStdDevs = kSingleTagStdDevs;
            int numTags = 0;
            double avgDist = 0;

            // Precalculation - see how many tags we found, and calculate an average-distance metric
            for (var tgt : targets) {
                var tagPose = photonEstimator.getFieldTags().getTagPose(tgt.getFiducialId());
                if (tagPose.isEmpty())
                    continue;
                numTags++;
                avgDist += tagPose.get().toPose2d().getTranslation().getDistance(estimatedPose.get().estimatedPose.toPose2d().getTranslation());
            }

            if (numTags == 0) {
                // No tags visible. Default to single-tag std devs
                curStdDevs = kSingleTagStdDevs;
            } else {
                // One or more tags visible, run the full heuristic.
                avgDist /= numTags;
                // Decrease std devs if multiple targets are visible
                if (numTags > 1)
                    estStdDevs = kMultiTagStdDevs;
                // Increase std devs based on (average) distance
                if (numTags == 1 && avgDist > 4)
                    estStdDevs = VecBuilder.fill(Double.MAX_VALUE, Double.MAX_VALUE, Double.MAX_VALUE);
                else
                    estStdDevs = estStdDevs.times(1 + (avgDist * avgDist / 30));
                curStdDevs = estStdDevs;
            }
        }
    }

    /**
     * Returns the latest standard deviations of the estimated pose from {@link #getEstimatedGlobalPose()}, for use with {@link edu.wpi.first.math.estimator.SwerveDrivePoseEstimator
     * SwerveDrivePoseEstimator}. This should only be used when there are targets visible.
     */
    public Matrix<N3, N1> getEstimationStdDevs() {
        return curStdDevs;
    }

    // ----- Simulation

    public void simulationPeriodic(Pose2d robotSimPose) {
        visionSim.update(robotSimPose);
    }

    /** Reset pose history of the robot in the vision system simulation. */
    public void resetSimPose(Pose2d pose) {
        if (Robot.isSimulation())
            visionSim.resetRobotPose(pose);
    }

    /** A Field2d for visualizing our robot and objects on the field. */
    public Field2d getSimDebugField() {
        if (!Robot.isSimulation())
            return null;
        return visionSim.getDebugField();
    }

    @FunctionalInterface
    public static interface EstimateConsumer {
        public void accept(Pose2d pose, double timestamp, Matrix<N3, N1> estimationStdDevs);
    }
}
