# Turret Calibration System Guide

## Overview

The turret calibration system allows operators to empirically determine the optimal hood angle and flywheel RPM settings for scoring from various distances on the field. Rather than relying on physics calculations or guesswork, the system lets you drive the robot to different positions, manually tune the hood and flywheel until shots consistently score, then save those settings as calibration points. These points are stored in a CSV file and processed at build time to generate lookup tables (`TurretLookupTables.java`) that the robot uses during matches to automatically select the correct settings based on distance to the target.

## How It Works

The calibration is distance-based: when you save a point, the system records your robot's position, the calculated distance to the target, and your tuned hood angle and flywheel RPM. Because distance is the key variable (not absolute field position), the calibration works for both alliances-a robot 5 meters from the blue target needs the same settings as a robot 5 meters from the red target. The `GenerateLookupTables` tool groups calibration points into 0.25-meter distance buckets, averages multiple measurements at similar distances, and generates sorted lookup tables for efficient runtime interpolation.

## Implementation

The system consists of four main components: `TurretCalibrationCommand` runs during test mode and publishes robot state (position, distance, actual hood/flywheel values) to NetworkTables while subscribing to user inputs for hood angle and flywheel RPM-these inputs are applied to the subsystems in real-time so you can see the effect immediately. `CalibrationDataRecorder` manages the in-memory buffer of calibration points and handles CSV file I/O on the roboRIO. The Elastic dashboard layout (`turret-calibration-layout.json`) provides the operator interface with input fields, status displays, and action buttons. Finally, `GenerateLookupTables.java` is a build-time tool that processes the CSV data into optimized Java lookup tables.

## How to Use

1. **Enter Test Mode**: Select "Test" in the Driver Station-this activates the calibration bindings.
2. **Start Calibration**: Press the right bumper on the driver controller to toggle calibration mode on.
3. **Load the Dashboard**: Import `elastic/turret-calibration-layout.json` into Elastic to see the calibration interface.
4. **Position the Robot**: Drive to a location where you want to calibrate (the dashboard shows your current grid position and suggests the next target location).
5. **Tune Settings**: Adjust the Hood Angle and Flywheel RPM inputs in Elastic until shots consistently score. The dashboard shows actual values and whether the hood/flywheel have reached their setpoints.
6. **Save the Point**: Click the "Save Point" button to record the current settings. The system auto-saves to CSV after each point.
7. **Repeat**: Move to different distances and repeat the tuning process. The grid system helps ensure coverage across the scoring zone.
8. **Export**: When finished, click "Export CSV" for a final save. The CSV file is stored at `src/main/deploy/calibration/turret_calibration_data.csv`.
9. **Generate Tables**: Run `./gradlew build`-the build process automatically runs `GenerateLookupTables` to create `TurretLookupTables.java` from your calibration data.
