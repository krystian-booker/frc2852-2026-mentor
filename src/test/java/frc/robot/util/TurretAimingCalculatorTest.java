package frc.robot.util;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

import org.junit.jupiter.api.BeforeAll;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

import edu.wpi.first.hal.HAL;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import frc.robot.Constants.TurretAimingConstants;

/**
 * Exercises the SOTM solve with a fake pose/speeds source. Alliance defaults
 * to Blue when no DriverStation is attached, so poses are placed in the Blue
 * scoring zone (x < 4.625) aiming at the Blue hub (4.625, 4.040).
 */
class TurretAimingCalculatorTest {

    private Pose2d pose = Pose2d.kZero;
    private ChassisSpeeds speeds = new ChassisSpeeds();
    private TurretAimingCalculator calculator;

    @BeforeAll
    static void initHal() {
        assertTrue(HAL.initialize(500, 0));
    }

    @BeforeEach
    void setup() {
        calculator = new TurretAimingCalculator(() -> pose, () -> speeds);
    }

    @Test
    void stationarySolveMatchesMapAtGeometricDistance() {
        // 2 m short of the hub, heading 0 — deep in the Blue zone
        pose = new Pose2d(2.625, 4.040, Rotation2d.kZero);
        speeds = new ChassisSpeeds();

        var solution = calculator.solve();

        assertTrue(solution.isHubShot());
        assertTrue(solution.isReachable());
        // Turret is offset behind/right of center, so distance is a bit more
        // than 2 m; hood/RPM must match the hub map at the solved distance
        assertEquals(calculator.getHubMap().getHoodDegrees(solution.distanceMeters()),
                solution.hoodAngleDegrees(), 1e-9);
        assertEquals(calculator.getHubMap().getFlywheelRPM(solution.distanceMeters()),
                solution.flywheelRPM(), 1e-9);
        assertTrue(solution.timeOfFlightSeconds() > 0.2 && solution.timeOfFlightSeconds() < 2.0);
        // Aiming nearly straight ahead
        assertTrue(Math.abs(solution.turretAngleDegrees()) < 10.0,
                "expected near-zero turret angle, got " + solution.turretAngleDegrees());
        // No motion: no lead, no bearing-rate feedforward
        assertEquals(0.0, solution.turretVelocityFFDegreesPerSecond(), 1e-9);
        assertEquals(TurretAimingConstants.BLUE_TARGET_POSITION, calculator.getLastTargetPosition());
    }

    @Test
    void movingSolveLeadsOppositeToMotion() {
        pose = new Pose2d(2.625, 4.040, Rotation2d.kZero);
        speeds = new ChassisSpeeds();
        double stationaryAngle = calculator.solve().turretAngleDegrees();

        // Strafing +Y at 2 m/s: the ball inherits +Y velocity, so the aim
        // point must shift toward -Y (turret angle decreases)
        speeds = new ChassisSpeeds(0.0, 2.0, 0.0);
        var moving = calculator.solve();

        assertTrue(moving.turretAngleDegrees() < stationaryAngle - 5.0,
                "expected significant lead, stationary=" + stationaryAngle
                        + " moving=" + moving.turretAngleDegrees());
        // Aim target displaced in -Y from the hub, never past the max lead clamp
        double leadY = TurretAimingConstants.BLUE_TARGET_POSITION.getY()
                - calculator.getLastTargetPosition().getY();
        assertTrue(leadY > 0.1 && leadY <= TurretAimingConstants.SOTM_MAX_LEAD_METERS + 1e-9,
                "lead was " + leadY);
        // Turret must slew to track the bearing change while strafing
        assertTrue(moving.turretVelocityFFDegreesPerSecond() < -10.0,
                "expected negative bearing-rate FF, got " + moving.turretVelocityFFDegreesPerSecond());
    }

    @Test
    void drivingTowardHubShortensEffectiveDistance() {
        pose = new Pose2d(1.625, 4.040, Rotation2d.kZero);
        speeds = new ChassisSpeeds();
        double stationaryDistance = calculator.solve().distanceMeters();

        // Driving toward the hub at 2 m/s: ball carries extra forward speed,
        // so we solve as if shooting from closer (virtual target pulled toward
        // the robot along -X... i.e. effective distance shrinks)
        speeds = new ChassisSpeeds(2.0, 0.0, 0.0);
        double movingDistance = calculator.solve().distanceMeters();

        assertTrue(movingDistance < stationaryDistance - 0.3,
                "expected shorter effective distance, stationary=" + stationaryDistance
                        + " moving=" + movingDistance);
    }

    @Test
    void pureRotationFeedforwardCounterRotates() {
        pose = new Pose2d(2.625, 4.040, Rotation2d.kZero);
        speeds = new ChassisSpeeds(0.0, 0.0, Math.toRadians(90.0)); // +90 deg/s yaw
        var solution = calculator.solve();
        // Robot-relative aim angle must change at -omega to hold field heading
        assertEquals(-90.0, solution.turretVelocityFFDegreesPerSecond(), 1.0);
    }

    @Test
    void outsideScoringZoneUsesLobMap() {
        // Neutral zone, lower half: Blue lobs to (1.5, 2.5)
        pose = new Pose2d(8.0, 2.0, Rotation2d.kZero);
        speeds = new ChassisSpeeds();
        var solution = calculator.solve();

        assertTrue(!solution.isHubShot());
        assertEquals(calculator.getLobMap().getHoodDegrees(solution.distanceMeters()),
                solution.hoodAngleDegrees(), 1e-9);
        // Lob target is behind the robot: large turret angle magnitude
        assertTrue(Math.abs(solution.turretAngleDegrees()) > 90.0);
    }

    @Test
    void invalidPoseYieldsSafeInvalidSolution() {
        pose = null;
        var solution = calculator.solve();
        assertTrue(!solution.isReachable());
        assertEquals(0.0, solution.flywheelRPM(), 1e-9);
    }

    @Test
    void zoneSelectionHasHysteresisAtBoundary() {
        // Start clearly inside the Blue zone: hub shot
        pose = new Pose2d(4.0, 4.0, Rotation2d.kZero);
        assertTrue(calculator.solve().isHubShot());

        // Drift just past the line, within the hysteresis band: still hub
        pose = new Pose2d(TurretAimingConstants.BLUE_ZONE_MAX_X + 0.1, 4.0, Rotation2d.kZero);
        assertTrue(calculator.solve().isHubShot(),
                "pose noise past the boundary must not flip the target");

        // Clearly past the band: switches to lob
        pose = new Pose2d(TurretAimingConstants.BLUE_ZONE_MAX_X
                + TurretAimingConstants.ZONE_HYSTERESIS_METERS + 0.1, 4.0, Rotation2d.kZero);
        assertTrue(!calculator.solve().isHubShot());

        // Re-entering flips back immediately at the line
        pose = new Pose2d(TurretAimingConstants.BLUE_ZONE_MAX_X - 0.1, 4.0, Rotation2d.kZero);
        assertTrue(calculator.solve().isHubShot());
    }

    @Test
    void lobSideSelectionHasCenterlineHysteresis() {
        // Neutral zone, below centerline: left lob target
        pose = new Pose2d(8.0, 2.0, Rotation2d.kZero);
        calculator.solve();
        assertEquals(TurretAimingConstants.BLUE_LEFT_TARGET_POSITION, calculator.getLastTargetPosition());

        // Just across the centerline, inside the band: still left
        pose = new Pose2d(8.0, TurretAimingConstants.FIELD_CENTERLINE_Y + 0.1, Rotation2d.kZero);
        calculator.solve();
        assertEquals(TurretAimingConstants.BLUE_LEFT_TARGET_POSITION, calculator.getLastTargetPosition());

        // Decisively across: right
        pose = new Pose2d(8.0, TurretAimingConstants.FIELD_CENTERLINE_Y
                + TurretAimingConstants.CENTERLINE_HYSTERESIS_METERS + 0.1, Rotation2d.kZero);
        calculator.solve();
        assertEquals(TurretAimingConstants.BLUE_RIGHT_TARGET_POSITION, calculator.getLastTargetPosition());

        // Coming back inside the band keeps right until decisively below
        pose = new Pose2d(8.0, TurretAimingConstants.FIELD_CENTERLINE_Y - 0.1, Rotation2d.kZero);
        calculator.solve();
        assertEquals(TurretAimingConstants.BLUE_RIGHT_TARGET_POSITION, calculator.getLastTargetPosition());
    }
}
