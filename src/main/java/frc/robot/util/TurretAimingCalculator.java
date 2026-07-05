package frc.robot.util;

import java.nio.file.Path;
import java.util.ArrayList;
import java.util.List;
import java.util.function.Supplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.Constants.ShooterModelConstants;
import frc.robot.Constants.TurretAimingConstants;
import frc.robot.util.ShotMap.ShotPoint;

/**
 * Computes a complete, coherent shot solution — turret angle, turret velocity
 * feedforward, hood angle, flywheel RPM, and time of flight — from the robot's
 * pose and velocity.
 *
 * <p>
 * Shot parameters come from distance-keyed {@link ShotMap}s (hub map inside
 * the scoring zone, lob map outside) instead of the old field-position grid.
 *
 * <p>
 * Shoot-on-the-move uses the standard virtual-target ("ghost goal") method:
 * the ball leaves the shooter carrying a fraction of the chassis velocity, so
 * we aim as if shooting stationary at a target displaced by
 * {@code -inheritedVelocity * timeOfFlight}. Time of flight depends on the
 * shot distance, which depends on the virtual target, so the pair is solved by
 * fixed-point iteration (converges in 2-3 rounds). The robot pose is also
 * projected forward by a small lookahead to cover pose latency and feed time.
 */
public class TurretAimingCalculator {
    private final Supplier<Pose2d> poseSupplier;
    private final Supplier<ChassisSpeeds> speedsSupplier;

    private final ShotMap hubMap;
    private final ShotMap lobMap;

    // Live SOTM tuning knobs (see SHOOTER_TUNING.md)
    private final TunableNumber velocityInheritance = new TunableNumber(
            "Shooter/SOTM/VelocityInheritance", ShooterModelConstants.SOTM_VELOCITY_INHERITANCE_DEFAULT);
    private final TunableNumber tofScale = new TunableNumber(
            "Shooter/SOTM/TofScale", ShooterModelConstants.SOTM_TOF_SCALE_DEFAULT);
    private final TunableNumber releaseLookahead = new TunableNumber(
            "Shooter/SOTM/ReleaseLookaheadSecs", ShooterModelConstants.SOTM_RELEASE_LOOKAHEAD_SECONDS_DEFAULT);

    // Alliance cache to prevent expensive DriverStation lookups
    private Alliance cachedAlliance = Alliance.Blue;
    private double lastAllianceCheckTime = -10.0;

    // Hysteresis state for target selection. Without a band around the zone
    // boundary and centerline, pose noise flips the selected target every
    // loop and the turret swings between very different bearings.
    private boolean inOwnScoringZone = false;
    private Boolean lobTargetOnRightSide = null;

    // Last solution, kept for telemetry/logging
    private ShotSolution lastSolution = ShotSolution.invalid();
    private Translation2d lastTargetPosition = new Translation2d();

    /**
     * A complete shot solution for the current robot state.
     *
     * @param turretAngleDegrees               Robot-relative turret setpoint,
     *                                         within the turret's range
     * @param turretVelocityFFDegreesPerSecond Rate the turret angle is changing
     *                                         due to chassis motion — feed to the
     *                                         turret as velocity feedforward
     * @param hoodAngleDegrees                 Hood mechanism setpoint
     * @param flywheelRPM                      Flywheel setpoint
     * @param distanceMeters                   Effective (virtual-target) shot
     *                                         distance
     * @param timeOfFlightSeconds              Estimated ball flight time
     * @param isHubShot                        True when aiming at the hub (robot
     *                                         in its scoring zone)
     * @param isReachable                      True when the shot distance is
     *                                         within the valid range
     */
    public record ShotSolution(
            double turretAngleDegrees,
            double turretVelocityFFDegreesPerSecond,
            double hoodAngleDegrees,
            double flywheelRPM,
            double distanceMeters,
            double timeOfFlightSeconds,
            boolean isHubShot,
            boolean isReachable) {

        static ShotSolution invalid() {
            return new ShotSolution(0, 0,
                    ShooterModelConstants.EMPTY_MAP_HOOD_DEGREES,
                    0, 0, 0, false, false);
        }
    }

    public TurretAimingCalculator(Supplier<Pose2d> poseSupplier, Supplier<ChassisSpeeds> speedsSupplier) {
        this.poseSupplier = poseSupplier;
        this.speedsSupplier = speedsSupplier;

        Path shotMapDir = Filesystem.getOperatingDirectory().toPath()
                .resolve(ShooterModelConstants.SHOTMAP_DIRECTORY);
        this.hubMap = new ShotMap("hub",
                ShooterModelConstants.HUB_RIM_HEIGHT_METERS,
                toPoints(ShooterModelConstants.HUB_DEFAULT_POINTS),
                shotMapDir.resolve("hub.csv"));
        this.lobMap = new ShotMap("lob",
                ShooterModelConstants.LOB_LANDING_HEIGHT_METERS,
                toPoints(ShooterModelConstants.LOB_DEFAULT_POINTS),
                shotMapDir.resolve("lob.csv"));
    }

    private static List<ShotPoint> toPoints(double[][] raw) {
        List<ShotPoint> list = new ArrayList<>(raw.length);
        for (double[] p : raw) {
            list.add(new ShotPoint(p[0], p[1], p[2]));
        }
        return list;
    }

    /** Returns the current robot pose from the pose supplier. */
    public Pose2d getRobotPose() {
        return poseSupplier.get();
    }

    public ShotMap getHubMap() {
        return hubMap;
    }

    public ShotMap getLobMap() {
        return lobMap;
    }

    /**
     * Solves the full shot for the current robot state. Cheap enough to call
     * every loop from multiple consumers; each call is computed fresh so the
     * turret command and shoot command always agree within a loop.
     */
    public ShotSolution solve() {
        Pose2d robotPose = poseSupplier.get();
        if (robotPose == null || Double.isNaN(robotPose.getX()) || Double.isNaN(robotPose.getY())) {
            lastSolution = ShotSolution.invalid();
            return lastSolution;
        }

        ChassisSpeeds fieldSpeeds = ChassisSpeeds.fromRobotRelativeSpeeds(
                speedsSupplier.get(), robotPose.getRotation());
        Translation2d fieldVelocity = new Translation2d(
                fieldSpeeds.vxMetersPerSecond, fieldSpeeds.vyMetersPerSecond);
        double chassisOmegaDegPerSec = Math.toDegrees(fieldSpeeds.omegaRadiansPerSecond);

        boolean isHubShot = isInOwnScoringZone(robotPose);
        ShotMap map = isHubShot ? hubMap : lobMap;
        Translation2d realTarget = getTargetPosition(robotPose);
        Translation2d turretPosition = getTurretFieldPosition(robotPose);

        boolean sotm = TurretAimingConstants.SOTM_ENABLED;
        double inheritance = sotm ? MathUtil.clamp(velocityInheritance.get(), 0.0, 1.0) : 0.0;

        // Project the shooter position to where it will be at ball release
        Translation2d releasePosition = sotm
                ? turretPosition.plus(fieldVelocity.times(Math.max(0.0, releaseLookahead.get())))
                : turretPosition;

        // Fixed-point solve: time of flight depends on the virtual-target
        // distance, which depends on time of flight
        Translation2d aimTarget = realTarget;
        double distance = releasePosition.getDistance(realTarget);
        double timeOfFlight = map.getTimeOfFlightSeconds(distance) * tofScale.get();
        if (inheritance > 0.0) {
            for (int i = 0; i < ShooterModelConstants.SOTM_ITERATIONS; i++) {
                Translation2d lead = fieldVelocity.times(inheritance * timeOfFlight);
                if (lead.getNorm() > TurretAimingConstants.SOTM_MAX_LEAD_METERS) {
                    lead = lead.times(TurretAimingConstants.SOTM_MAX_LEAD_METERS / lead.getNorm());
                }
                aimTarget = realTarget.minus(lead);
                distance = releasePosition.getDistance(aimTarget);
                timeOfFlight = map.getTimeOfFlightSeconds(distance) * tofScale.get();
            }
        }

        double turretAngleDegrees = calculateTurretAngleToTarget(robotPose, aimTarget);

        // Velocity feedforward: rate of change of the robot-relative turret
        // angle. Bearing to the target changes as the robot translates
        // (d/dt atan2 = (r x v)/|r|^2) and the robot-relative angle also
        // counter-rotates against chassis yaw.
        Translation2d toTarget = aimTarget.minus(turretPosition);
        double rangeSquared = Math.max(toTarget.getNorm() * toTarget.getNorm(), 1e-6);
        double bearingRateDegPerSec = Math.toDegrees(
                (toTarget.getY() * fieldVelocity.getX() - toTarget.getX() * fieldVelocity.getY()) / rangeSquared);
        double turretVelocityFF = bearingRateDegPerSec - chassisOmegaDegPerSec;

        boolean isReachable = distance >= TurretAimingConstants.MIN_SHOOTING_DISTANCE_METERS
                && distance <= TurretAimingConstants.MAX_SHOOTING_DISTANCE_METERS;

        lastTargetPosition = aimTarget;
        lastSolution = new ShotSolution(
                turretAngleDegrees,
                turretVelocityFF,
                map.getHoodDegrees(distance),
                map.getFlywheelRPM(distance),
                distance,
                timeOfFlight,
                isHubShot,
                isReachable);
        return lastSolution;
    }

    /**
     * Stationary distance and map selection for calibration mode — no SOTM
     * lead, no lookahead, measured from the turret to the real target.
     *
     * @return {distanceMeters, isHubShot} for the robot's current position
     */
    public record CalibrationTarget(double distanceMeters, boolean isHubShot) {
    }

    public CalibrationTarget getCalibrationTarget() {
        Pose2d robotPose = poseSupplier.get();
        if (robotPose == null || Double.isNaN(robotPose.getX()) || Double.isNaN(robotPose.getY())) {
            return new CalibrationTarget(0.0, false);
        }
        Translation2d target = getTargetPosition(robotPose);
        double distance = getTurretFieldPosition(robotPose).getDistance(target);
        return new CalibrationTarget(distance, isInOwnScoringZone(robotPose));
    }

    /**
     * True when the robot is inside its alliance's scoring zone. Entering is
     * decided exactly at the boundary; leaving requires driving
     * {@link TurretAimingConstants#ZONE_HYSTERESIS_METERS} past it, so pose
     * noise at the line can't rapidly flip the turret between the hub and the
     * lob targets.
     */
    public boolean isInOwnScoringZone(Pose2d robotPose) {
        // Positive depth = inside own zone
        double depth = getAlliance() == Alliance.Blue
                ? TurretAimingConstants.BLUE_ZONE_MAX_X - robotPose.getX()
                : robotPose.getX() - TurretAimingConstants.RED_ZONE_MIN_X;
        if (inOwnScoringZone) {
            inOwnScoringZone = depth > -TurretAimingConstants.ZONE_HYSTERESIS_METERS;
        } else {
            inOwnScoringZone = depth > 0.0;
        }
        return inOwnScoringZone;
    }

    /**
     * Gets the target position based on the current alliance and robot position on
     * the field.
     *
     * <p>
     * When the robot is in its own scoring zone, aims at the alliance goal. When in
     * the neutral or opponent zone, aims at alliance-specific left/right targets
     * based on robot Y position relative to the field centerline.
     *
     * @param robotPose The robot's current pose
     * @return Target position in field coordinates (meters)
     */
    public Translation2d getTargetPosition(Pose2d robotPose) {
        boolean blue = getAlliance() == Alliance.Blue;
        if (isInOwnScoringZone(robotPose)) {
            return blue ? TurretAimingConstants.BLUE_TARGET_POSITION
                    : TurretAimingConstants.RED_TARGET_POSITION;
        }
        if (isLobTargetOnRightSide(robotPose)) {
            return blue ? TurretAimingConstants.BLUE_RIGHT_TARGET_POSITION
                    : TurretAimingConstants.RED_RIGHT_TARGET_POSITION;
        }
        return blue ? TurretAimingConstants.BLUE_LEFT_TARGET_POSITION
                : TurretAimingConstants.RED_LEFT_TARGET_POSITION;
    }

    /**
     * Chooses the left/right lob target with hysteresis around the field
     * centerline, so driving along the centerline doesn't flip the turret
     * between the two targets every loop.
     */
    private boolean isLobTargetOnRightSide(Pose2d robotPose) {
        double y = robotPose.getY();
        if (lobTargetOnRightSide == null) {
            lobTargetOnRightSide = y >= TurretAimingConstants.FIELD_CENTERLINE_Y;
        } else if (lobTargetOnRightSide) {
            lobTargetOnRightSide = y > TurretAimingConstants.FIELD_CENTERLINE_Y
                    - TurretAimingConstants.CENTERLINE_HYSTERESIS_METERS;
        } else {
            lobTargetOnRightSide = y > TurretAimingConstants.FIELD_CENTERLINE_Y
                    + TurretAimingConstants.CENTERLINE_HYSTERESIS_METERS;
        }
        return lobTargetOnRightSide;
    }

    /** Returns the aim target position used in the last solve. */
    public Translation2d getLastTargetPosition() {
        return lastTargetPosition;
    }

    /** Returns the effective shot distance from the last solve. */
    public double getLastDistanceMeters() {
        return lastSolution.distanceMeters();
    }

    /** Returns the full solution from the last solve, for telemetry. */
    public ShotSolution getLastSolution() {
        return lastSolution;
    }

    /**
     * Calculates the turret field position accounting for the mechanical offset
     * from robot center.
     */
    private Translation2d getTurretFieldPosition(Pose2d robotPose) {
        return robotPose.getTranslation()
                .plus(new Translation2d(
                        TurretAimingConstants.TURRET_OFFSET_X_METERS,
                        TurretAimingConstants.TURRET_OFFSET_Y_METERS)
                        .rotateBy(robotPose.getRotation()));
    }

    /**
     * Calculates the robot-relative turret angle to aim at a given target.
     * Includes turret offset, angle normalization, and range wrapping.
     *
     * @param robotPose      The robot's current pose
     * @param targetPosition The target to aim at (field coordinates)
     * @return Turret angle in degrees, within the turret's commandable range
     */
    private double calculateTurretAngleToTarget(Pose2d robotPose, Translation2d targetPosition) {
        Translation2d turretPosition = getTurretFieldPosition(robotPose);

        // Field-relative bearing to the target
        double fieldAngleRadians = Math.atan2(
                targetPosition.getY() - turretPosition.getY(),
                targetPosition.getX() - turretPosition.getX());

        // Robot-relative angle (subtract robot heading), wrapped into range
        double robotRelativeRadians = fieldAngleRadians - robotPose.getRotation().getRadians();
        return TurretAngles.normalizeToTurretRange(Math.toDegrees(robotRelativeRadians));
    }

    /**
     * Retrieves the current alliance from the DriverStation, but throttles
     * queries to only once per second. Calling DriverStation.getAlliance()
     * frequently is a leading cause of 20ms loop overruns.
     */
    private Alliance getAlliance() {
        double currentTime = Timer.getFPGATimestamp();
        if (currentTime - lastAllianceCheckTime > 1.0) {
            cachedAlliance = DriverStation.getAlliance().orElse(cachedAlliance);
            lastAllianceCheckTime = currentTime;
        }
        return cachedAlliance;
    }
}
