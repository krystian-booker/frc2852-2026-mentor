package frc.robot.util.firecontrol;

/**
 * Shared physics constants for the shooter system. Used by both the build-time LUT generator
 * (tools/GenerateShotLUT.java) and the robot runtime (Constants.BallSimConstants, BallSimManager).
 *
 * <p>IMPORTANT: This class must have ZERO imports so it can be compiled standalone by the LUT
 * generator Gradle task. Do not add any WPILib or other dependencies.
 *
 * <p>If you change values here, regenerate the LUT: {@code ./gradlew generateShotLUT}
 */
public final class ShooterPhysicsConstants {

  private ShooterPhysicsConstants() {}

  // Ball properties (per game manual)
  public static final double BALL_MASS_KG = 0.215; // game manual 5.10.1 midpoint (0.203-0.227 kg)
  public static final double BALL_DIAMETER_M = 0.150114; // 5.91 inches per game manual

  // Aerodynamics
  public static final double DRAG_COEFF = 0.47; // sphere drag coefficient
  public static final double MAGNUS_COEFF = 0.2;
  public static final double AIR_DENSITY = 1.225; // kg/m^3 at sea level
  public static final double MAGNUS_SIGN = 1.0; // +1 topspin

  // Launcher geometry
  public static final double SLIP_FACTOR = 0.6; // wheel-to-ball velocity coupling
  public static final double WHEEL_DIAMETER_M = 0.1016; // 4 inches
  public static final double EXIT_HEIGHT_M = 0.5; // launcher exit height above floor

  // Target
  public static final double TARGET_HEIGHT_M = 2.64; // scoring target height

  // Hood angle mapping (must match HoodConstants)
  public static final double ACTUAL_ANGLE_AT_ZERO = 70.0; // elevation when hood mechanism is at 0
  public static final double MECHANISM_MIN = 0.0;
  public static final double MECHANISM_MAX = 25.0;
}
