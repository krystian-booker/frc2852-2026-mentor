package frc.robot.subsystems.vision;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;

public class VisionConstants {
  // AprilTag layout
  public static final AprilTagFieldLayout aprilTagLayout =
      AprilTagFieldLayout.loadField(AprilTagFields.kDefaultField);

  // Camera names, must match names configured on coprocessor
  public static final String leftCameraName = "LEFT_CAMERA";
  public static final String rightCameraName = "RIGHT_CAMERA";

  // Robot to camera transforms
  public static final Transform3d robotToLeftCamera =
      new Transform3d(
          new Translation3d(0.0244348, 0.2737866, 0.6747256),
          new Rotation3d(Math.toRadians(0), Math.toRadians(-30), Math.toRadians(-45)));

  public static final Transform3d robotToRightCamera =
      new Transform3d(
          new Translation3d(0.0394716, -0.263906, 0.6851142),
          new Rotation3d(Math.toRadians(0), Math.toRadians(-30), Math.toRadians(45)));

  // Basic filtering thresholds
  public static final double maxAmbiguity = 0.3;
  public static final double maxZError = 0.75;

  // Standard deviation baselines, for 1 meter distance and 1 tag
  // (Adjusted automatically based on distance and # of tags)
  public static final double linearStdDevBaseline = 0.02; // Meters
  public static final double angularStdDevBaseline = 0.06; // Radians

  // Standard deviation multipliers for each camera
  // (Adjust to trust some cameras more than others)
  public static final double[] cameraStdDevFactors =
      new double[] {
        1.0, // Left camera
        1.0 // Right camera
      };
}
