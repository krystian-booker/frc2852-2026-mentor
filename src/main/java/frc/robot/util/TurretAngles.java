package frc.robot.util;

import frc.robot.Constants.TurretConstants;

/**
 * Angle math for the turret's wrapped command space. The turret can rotate
 * exactly one revolution ({@link TurretConstants#MIN_POSITION_DEGREES} to
 * {@link TurretConstants#MAX_POSITION_DEGREES}), so every physical direction
 * has a single in-range representation — except at the range seam, where the
 * same direction is reachable from both limits and choosing the wrong one
 * commands a full-revolution unwind.
 */
public final class TurretAngles {

    private TurretAngles() {
    }

    /**
     * Normalizes any angle into the turret's commandable range.
     *
     * @param degrees Robot-relative angle, any magnitude
     * @return Equivalent angle within [MIN_POSITION_DEGREES, MAX_POSITION_DEGREES]
     */
    public static double normalizeToTurretRange(double degrees) {
        // Normalize to (-180, +180]
        double normalized = degrees % 360.0;
        if (normalized > 180.0) {
            normalized -= 360.0;
        } else if (normalized <= -180.0) {
            normalized += 360.0;
        }
        // Shift into the turret range when a forward-offset skews it off (-180, 180]
        if (normalized > TurretConstants.MAX_POSITION_DEGREES) {
            normalized -= 360.0;
        } else if (normalized < TurretConstants.MIN_POSITION_DEGREES) {
            normalized += 360.0;
        }
        return normalized;
    }

    /**
     * True when moving from {@code fromDegrees} to {@code toDegrees} is a
     * near-full-revolution unwind whose actual bearing change is smaller than
     * {@link TurretConstants#WRAP_HYSTERESIS_DEGREES} — i.e. a noise-induced
     * flip across the range seam rather than a genuine new target direction.
     * A genuine large slew (bearing change bigger than the hysteresis) is not
     * a wrap flip even when it must travel more than 180°.
     */
    public static boolean isWrapFlip(double fromDegrees, double toDegrees) {
        double travel = Math.abs(toDegrees - fromDegrees);
        return travel > 360.0 - TurretConstants.WRAP_HYSTERESIS_DEGREES;
    }
}
