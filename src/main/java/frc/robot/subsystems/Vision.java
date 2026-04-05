package frc.robot.subsystems;

import static frc.robot.Constants.Vision.*;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.Vision.CameraConfig;
import frc.robot.Robot;
import java.util.ArrayList;
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

public class Vision extends SubsystemBase {
  /** Inner class representing a single camera instance with its own pose estimator and state. */
  private static class CameraInstance {
    final String name;
    final PhotonCamera camera;
    final PhotonPoseEstimator poseEstimator;

    @SuppressWarnings("unused")
    final Transform3d robotToCam;

    Matrix<N3, N1> currentStdDevs = kSingleTagStdDevs;

    @SuppressWarnings("unused")
    Pose2d latestEstimate = new Pose2d();

    @SuppressWarnings("unused")
    double latestTimestamp = 0.0;

    final Set<Integer> visibleTagIds = new HashSet<>();

    // For simulation
    PhotonCameraSim cameraSim;

    CameraInstance(String name, Transform3d robotToCam) {
      this.name = name;
      this.robotToCam = robotToCam;
      this.camera = new PhotonCamera(name);
      this.poseEstimator = new PhotonPoseEstimator(kTagLayout, robotToCam);
    }
  }

  private final List<CameraInstance> cameras = new ArrayList<>();
  private final Set<Integer> aggregatedVisibleTagIds = new HashSet<>();
  private final EstimateConsumer estConsumer;

  // State tracking for best pose estimation across all cameras
  private Matrix<N3, N1> curStdDevs = kSingleTagStdDevs;
  private Pose2d latestEstimate = new Pose2d();
  private double latestEstimateTimestamp = 0.0;

  // Feeding control - can be disabled after QuestNav seeding
  private boolean feedingEnabled = true;

  // Simulation - shared across all cameras
  private VisionSystemSim visionSim;

  /**
   * @param estConsumer Lambda that will accept a pose estimate and pass it to your desired {@link
   *     edu.wpi.first.math.estimator.SwerveDrivePoseEstimator}
   */
  public Vision(EstimateConsumer estConsumer) {
    this.estConsumer = estConsumer;

    // Initialize simulation first (if simulating) so cameras can be added to it
    if (Robot.isSimulation()) {
      // Disable PhotonVision version checking in simulation to prevent
      // "coprocessor is not sending new data" warnings
      PhotonCamera.setVersionCheckEnabled(false);

      visionSim = new VisionSystemSim("main");
      visionSim.addAprilTags(kTagLayout);
    }

    // Create camera instances from configuration
    for (CameraConfig config : CAMERAS) {
      CameraInstance camInstance = new CameraInstance(config.name(), config.robotToCam());
      cameras.add(camInstance);

      // Setup simulation for this camera
      if (Robot.isSimulation()) {
        var cameraProp = new SimCameraProperties();
        cameraProp.setCalibration(960, 720, Rotation2d.fromDegrees(90));
        cameraProp.setCalibError(0.35, 0.10);
        cameraProp.setFPS(15);
        cameraProp.setAvgLatencyMs(50);
        cameraProp.setLatencyStdDevMs(15);

        camInstance.cameraSim = new PhotonCameraSim(camInstance.camera, cameraProp);
        visionSim.addCamera(camInstance.cameraSim, config.robotToCam());
        camInstance.cameraSim.enableDrawWireframe(true);
      }
    }
  }

  @Override
  public void periodic() {
    // Clear aggregated visible tags for this frame
    aggregatedVisibleTagIds.clear();

    // Track the best estimate across all cameras (lowest std dev sum wins)
    CameraInstance bestCamera = null;
    double bestStdDevSum = Double.MAX_VALUE;
    EstimatedRobotPose bestEstimate = null;
    int bestNumTags = 0;

    // Process each camera
    for (CameraInstance cam : cameras) {
      cam.visibleTagIds.clear();

      for (var result : cam.camera.getAllUnreadResults()) {
        // Track visible tags for this camera
        for (var target : result.getTargets()) {
          cam.visibleTagIds.add(target.getFiducialId());
          aggregatedVisibleTagIds.add(target.getFiducialId());
        }

        // Estimate pose - prefer multi-tag, fall back to lowest ambiguity
        Optional<EstimatedRobotPose> visionEst =
            cam.poseEstimator.estimateCoprocMultiTagPose(result);
        if (visionEst.isEmpty()) {
          visionEst = cam.poseEstimator.estimateLowestAmbiguityPose(result);
        }

        // Calculate standard deviations for this camera's estimate
        updateCameraEstimationStdDevs(cam, visionEst, result.getTargets());

        if (visionEst.isPresent()) {
          // Update camera's latest estimate
          cam.latestEstimate = visionEst.get().estimatedPose.toPose2d();
          cam.latestTimestamp = visionEst.get().timestampSeconds;

          // Calculate confidence score (sum of std devs - lower is better)
          double stdDevSum =
              cam.currentStdDevs.get(0, 0)
                  + cam.currentStdDevs.get(1, 0)
                  + cam.currentStdDevs.get(2, 0);

          // Check if this is the best estimate so far
          if (stdDevSum < bestStdDevSum) {
            bestStdDevSum = stdDevSum;
            bestCamera = cam;
            bestEstimate = visionEst.get();
            bestNumTags = result.getTargets().size();
          }
        }
      }

      // Per-camera telemetry
      // SmartDashboard.putNumber("Vision/" + cam.name + "/TagCount",
      // cam.visibleTagIds.size());
      // SmartDashboard.putString("Vision/" + cam.name + "/VisibleTags",
      // cam.visibleTagIds.toString());
    }

    // Use the best estimate if we have one
    if (bestCamera != null && bestEstimate != null) {
      latestEstimate = bestEstimate.estimatedPose.toPose2d();
      latestEstimateTimestamp = bestEstimate.timestampSeconds;
      curStdDevs = bestCamera.currentStdDevs;

      // Simulation debug visualization
      if (Robot.isSimulation()) {
        getSimDebugField().getObject("VisionEstimation").setPose(latestEstimate);
      }

      // Feed to drivetrain if enabled and we see multiple tags for better accuracy
      if (feedingEnabled && bestNumTags >= 2) {
        estConsumer.accept(latestEstimate, latestEstimateTimestamp, curStdDevs);
      }
    } else if (Robot.isSimulation()) {
      getSimDebugField().getObject("VisionEstimation").setPoses();
    }

    // Aggregated telemetry (backward compatible)
    // SmartDashboard.putBoolean("Vision/HasValidPose",
    // hasRecentValidPose(POSE_VALIDITY_TIMEOUT));
    // SmartDashboard.putString("Vision/VisibleTags",
    // aggregatedVisibleTagIds.toString());
    // SmartDashboard.putNumber("Vision/TotalVisibleTagCount",
    // aggregatedVisibleTagIds.size());
    // SmartDashboard.putNumber("Vision/PoseX", latestEstimate.getX());
    // SmartDashboard.putNumber("Vision/PoseY", latestEstimate.getY());
    // SmartDashboard.putNumber("Vision/PoseRotation",
    // latestEstimate.getRotation().getDegrees());
    // SmartDashboard.putBoolean("Vision/FeedingEnabled", feedingEnabled);
    // SmartDashboard.putString("Vision/BestCamera", bestCamera != null ?
    // bestCamera.name : "none");
  }

  /** Calculates standard deviations for a specific camera's pose estimate. */
  private void updateCameraEstimationStdDevs(
      CameraInstance cam,
      Optional<EstimatedRobotPose> estimatedPose,
      List<PhotonTrackedTarget> targets) {
    if (estimatedPose.isEmpty()) {
      cam.currentStdDevs = kSingleTagStdDevs;
      return;
    }

    var estStdDevs = kSingleTagStdDevs;
    int numTags = 0;
    double avgDist = 0;

    for (var tgt : targets) {
      var tagPose = cam.poseEstimator.getFieldTags().getTagPose(tgt.getFiducialId());
      if (tagPose.isEmpty()) continue;
      numTags++;
      avgDist +=
          tagPose
              .get()
              .toPose2d()
              .getTranslation()
              .getDistance(estimatedPose.get().estimatedPose.toPose2d().getTranslation());
    }

    if (numTags == 0) {
      cam.currentStdDevs = kSingleTagStdDevs;
    } else {
      avgDist /= numTags;
      if (numTags > 1) estStdDevs = kMultiTagStdDevs;
      if (numTags == 1 && avgDist > 4)
        estStdDevs = VecBuilder.fill(Double.MAX_VALUE, Double.MAX_VALUE, Double.MAX_VALUE);
      else estStdDevs = estStdDevs.times(1 + (avgDist * avgDist / 30));
      cam.currentStdDevs = estStdDevs;
    }
  }

  /**
   * Returns the latest estimated pose from vision (best estimate across all cameras).
   *
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
   *
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
   *
   * @return Timestamp in seconds, or 0 if no estimate exists
   */
  public double getLatestEstimateTimestamp() {
    return latestEstimateTimestamp;
  }

  /**
   * Returns the set of currently visible AprilTag IDs across all cameras.
   *
   * @return Set of visible tag IDs
   */
  public Set<Integer> getVisibleTagIds() {
    return new HashSet<>(aggregatedVisibleTagIds);
  }

  /**
   * Checks if a specific AprilTag is currently visible by any camera.
   *
   * @param tagId The tag ID to check
   * @return true if the tag is currently visible
   */
  public boolean isTagVisible(int tagId) {
    return aggregatedVisibleTagIds.contains(tagId);
  }

  /**
   * Returns the count of currently visible AprilTags across all cameras.
   *
   * @return Number of visible tags (union of all cameras)
   */
  public int getVisibleTagCount() {
    return aggregatedVisibleTagIds.size();
  }

  /**
   * Returns the count of visible AprilTags for a specific camera.
   *
   * @param cameraName The name of the camera
   * @return Number of visible tags for that camera, or 0 if camera not found
   */
  public int getVisibleTagCountForCamera(String cameraName) {
    for (CameraInstance cam : cameras) {
      if (cam.name.equals(cameraName)) {
        return cam.visibleTagIds.size();
      }
    }
    return 0;
  }

  /**
   * Returns the names of cameras that currently have visible tags.
   *
   * @return List of camera names with visible tags
   */
  public List<String> getCamerasWithVisibleTags() {
    List<String> result = new ArrayList<>();
    for (CameraInstance cam : cameras) {
      if (!cam.visibleTagIds.isEmpty()) {
        result.add(cam.name);
      }
    }
    return result;
  }

  /**
   * Sets whether vision should feed pose estimates to the drivetrain.
   *
   * @param enabled true to enable feeding, false to disable
   */
  public void setFeedingEnabled(boolean enabled) {
    this.feedingEnabled = enabled;
  }

  /**
   * Returns whether vision is currently feeding pose estimates to the drivetrain.
   *
   * @return true if feeding is enabled
   */
  public boolean isFeedingEnabled() {
    return feedingEnabled;
  }

  /**
   * Returns the latest standard deviations of the estimated pose, for use with {@link
   * edu.wpi.first.math.estimator.SwerveDrivePoseEstimator SwerveDrivePoseEstimator}. This should
   * only be used when there are targets visible.
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
    if (Robot.isSimulation()) visionSim.resetRobotPose(pose);
  }

  /** A Field2d for visualizing our robot and objects on the field. */
  public Field2d getSimDebugField() {
    if (!Robot.isSimulation()) return null;
    return visionSim.getDebugField();
  }

  @FunctionalInterface
  public static interface EstimateConsumer {
    public void accept(Pose2d pose, double timestamp, Matrix<N3, N1> estimationStdDevs);
  }
}
