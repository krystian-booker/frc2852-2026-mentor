package frc.robot.util;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import org.junit.jupiter.api.Test;

class TurretAnglesTest {

    @Test
    void normalizeLeavesInRangeAnglesAlone() {
        assertEquals(0.0, TurretAngles.normalizeToTurretRange(0.0), 1e-9);
        assertEquals(45.0, TurretAngles.normalizeToTurretRange(45.0), 1e-9);
        assertEquals(-135.0, TurretAngles.normalizeToTurretRange(-135.0), 1e-9);
        assertEquals(180.0, TurretAngles.normalizeToTurretRange(180.0), 1e-9);
    }

    @Test
    void normalizeWrapsOutOfRangeAngles() {
        assertEquals(-170.0, TurretAngles.normalizeToTurretRange(190.0), 1e-9);
        assertEquals(170.0, TurretAngles.normalizeToTurretRange(-190.0), 1e-9);
        assertEquals(1.0, TurretAngles.normalizeToTurretRange(361.0), 1e-9);
        assertEquals(-1.0, TurretAngles.normalizeToTurretRange(-721.0), 1e-9);
        assertEquals(180.0, TurretAngles.normalizeToTurretRange(-180.0), 1e-9);
    }

    @Test
    void noiseFlipAcrossSeamIsAWrapFlip() {
        // Bearing jitters across the +/-180 seam: 2 degrees of physical change
        // would command a 358 degree unwind
        assertTrue(TurretAngles.isWrapFlip(179.0, -179.0));
        assertTrue(TurretAngles.isWrapFlip(-179.0, 179.0));
        assertTrue(TurretAngles.isWrapFlip(-176.0, 178.0));
    }

    @Test
    void genuineSlewsAreNotWrapFlips() {
        // Normal tracking moves
        assertFalse(TurretAngles.isWrapFlip(0.0, 45.0));
        assertFalse(TurretAngles.isWrapFlip(-90.0, 90.0));
        // Big but real: 330 degrees of travel for 30 degrees of bearing change
        // is past the 10 degree hysteresis, so the unwind is legitimate
        assertFalse(TurretAngles.isWrapFlip(170.0, -160.0));
        // Target flip hub -> lob behind the robot
        assertFalse(TurretAngles.isWrapFlip(10.0, -170.0));
    }
}
