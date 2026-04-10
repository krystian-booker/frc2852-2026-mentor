"""
Recalculate turret calibration CSV after lowering the turret by 2.39 inches.

Physics: By adjusting both hood angle and RPM to maintain the same time-of-flight,
ball speed terms cancel. The formulas are:
    theta_new = atan(tan(theta_old) + delta_h / d)
    RPM_new   = RPM_old * cos(theta_old) / cos(theta_new)

Where theta is the ACTUAL launch elevation (70 - mechanism_angle), d is horizontal
distance to target, and delta_h is the height drop in meters.

Usage: python recalculate_turret_table.py
  - Reads:  src/main/deploy/calibration/turret_calibration_data.csv
  - Writes: src/main/deploy/calibration/turret_calibration_data.csv (overwrites)
  - Then run a Gradle build to regenerate TurretLookupTables.java
"""

import csv
import math
import os

# ── Configuration ────────────────────────────────────────────────────────────
HEIGHT_DROP_INCHES = 2.39
HEIGHT_DROP_METERS = HEIGHT_DROP_INCHES * 0.0254  # 0.060706 m

BLUE_TARGET = (4.625, 4.040)  # field coordinates in meters

ACTUAL_ANGLE_AT_ZERO = 70.0  # mechanism position 0 = 70 deg actual elevation
HOOD_MIN = 0.0
HOOD_MAX = 25.0
RPM_MIN = 1000
RPM_MAX = 4700

MIN_DISTANCE = 0.25  # skip rows closer than this to avoid extreme corrections

CSV_PATH = os.path.join("src", "main", "deploy", "calibration", "turret_calibration_data.csv")


def recalculate_row(robot_x, robot_y, old_hood, old_rpm):
    """Apply height-drop correction to a single calibration data point."""
    dh = HEIGHT_DROP_METERS
    target_x, target_y = BLUE_TARGET

    dx = target_x - robot_x
    dy = target_y - robot_y
    d = math.sqrt(dx * dx + dy * dy)

    if d < MIN_DISTANCE:
        return old_hood, old_rpm, 0.0, 0

    # Convert mechanism angle to actual launch elevation
    theta_old_deg = ACTUAL_ANGLE_AT_ZERO - old_hood
    theta_old = math.radians(theta_old_deg)

    # New actual angle: tan(theta_new) = tan(theta_old) + dh / d
    tan_new = math.tan(theta_old) + dh / d
    theta_new = math.atan(tan_new)
    theta_new_deg = math.degrees(theta_new)

    # Convert back to mechanism angle
    new_hood = ACTUAL_ANGLE_AT_ZERO - theta_new_deg
    new_hood = max(HOOD_MIN, min(HOOD_MAX, new_hood))
    new_hood = round(new_hood, 2)

    # Adjust RPM to maintain same time-of-flight
    cos_old = math.cos(theta_old)
    cos_new = math.cos(theta_new)
    if cos_new > 0.001:
        new_rpm = old_rpm * cos_old / cos_new
    else:
        new_rpm = old_rpm
    new_rpm = max(RPM_MIN, min(RPM_MAX, new_rpm))
    new_rpm = round(new_rpm)

    return new_hood, new_rpm, new_hood - old_hood, new_rpm - old_rpm


def main():
    # Read the CSV
    with open(CSV_PATH, "r", newline="") as f:
        reader = csv.DictReader(f)
        fieldnames = reader.fieldnames
        rows = list(reader)

    print(f"Read {len(rows)} rows from {CSV_PATH}")
    print(f"Turret height lowered by {HEIGHT_DROP_INCHES}\" ({HEIGHT_DROP_METERS:.4f} m)")
    print(f"Target position: ({BLUE_TARGET[0]}, {BLUE_TARGET[1]})")
    print()

    # Process each row
    updated_rows = []
    changes = []

    for row in rows:
        robot_x = float(row["robot_x"])
        robot_y = float(row["robot_y"])
        old_hood = float(row["hood_angle_degrees"])
        old_rpm = int(float(row["flywheel_rpm"]))
        grid_row = int(row["grid_row"])
        grid_col = int(row["grid_col"])

        new_hood, new_rpm, d_hood, d_rpm = recalculate_row(
            robot_x, robot_y, old_hood, old_rpm
        )

        row["hood_angle_degrees"] = f"{new_hood:.2f}"
        row["flywheel_rpm"] = str(new_rpm)
        updated_rows.append(row)

        if abs(d_hood) >= 0.01 or abs(d_rpm) >= 1:
            dist = math.sqrt((BLUE_TARGET[0] - robot_x)**2 + (BLUE_TARGET[1] - robot_y)**2)
            changes.append((grid_row, grid_col, dist, old_hood, new_hood, d_hood,
                            old_rpm, new_rpm, d_rpm))

    # Print validation cell [8][7]
    print(">>> VALIDATION CELL [8][7] <<<")
    for gr, gc, d, oh, nh, dh, orpm, nrpm, dr in changes:
        if gr == 8 and gc == 7:
            print(f"  Distance to target: {d:.3f} m")
            print(f"  Hood angle: {oh:.2f} -> {nh:.2f} (delta: {dh:+.2f})")
            print(f"  RPM:        {orpm} -> {nrpm} (delta: {dr:+d})")
            break
    else:
        print("  (not found in changes)")

    # Print summary of biggest changes
    print(f"\nCells changed: {len(changes)}")
    print(f"\n{'Row':>3} {'Col':>3} {'Dist':>6} | {'Old Hood':>8} {'New Hood':>8} {'dHood':>7} | {'Old RPM':>7} {'New RPM':>7} {'dRPM':>6}")
    print("-" * 90)
    for gr, gc, d, oh, nh, dh, orpm, nrpm, dr in sorted(changes, key=lambda c: -abs(c[5]))[:30]:
        marker = " <<<" if (gr == 8 and gc == 7) else ""
        print(f"{gr:>3} {gc:>3} {d:>6.2f} | {oh:>8.2f} {nh:>8.2f} {dh:>+7.2f} | {orpm:>7} {nrpm:>7} {dr:>+6d}{marker}")
    if len(changes) > 30:
        print(f"  ... and {len(changes) - 30} more cells with smaller changes")

    # Write updated CSV
    with open(CSV_PATH, "w", newline="") as f:
        writer = csv.DictWriter(f, fieldnames=fieldnames)
        writer.writeheader()
        writer.writerows(updated_rows)

    print(f"\nWrote {len(updated_rows)} rows to {CSV_PATH}")
    print("Run a Gradle build to regenerate TurretLookupTables.java")


if __name__ == "__main__":
    main()
