# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project Overview

FRC Team 2852 robot code for the 2026 season. Built on WPILib's command-based framework with CTRE Phoenix6 hardware, swerve drivetrain, and a multi-subsystem shooter/aiming system.

## Build & Deploy Commands

```bash
./gradlew build              # Build (also generates turret lookup tables from CSV)
./gradlew deploy             # Deploy to RoboRIO
./gradlew test               # Run JUnit 5 tests
./gradlew simulateJava       # Run robot simulation with GUI
./gradlew generateTurretLookupTables  # Regenerate lookup tables from calibration CSV
```

Java 17 is required. Gradle wrapper is included (`gradlew`/`gradlew.bat`).

## Architecture

**Command-based robot** using WPILib's SubsystemBase/Command pattern. Entry point flows: `Main.java` → `Robot.java` (TimedRobot lifecycle) → `RobotContainer.java` (subsystem initialization and controller bindings).

### Key subsystems (`subsystems/`)
- **CommandSwerveDrivetrain** - CTRE TalonFX swerve drive, auto-generated from Phoenix Tuner. Uses CANivore bus.
- **Flywheel, Hood, Turret** - Shooter aiming system. Turret auto-aims using vision and lookup tables.
- **Conveyor, Intake, IntakeActuator** - Game piece handling chain.
- **Vision** - Dual-camera AprilTag detection via photonlib.
- **QuestNavSubsystem** - Meta Quest 3 headset for visual odometry.

### Commands (`commands/`)
- **ShootCommand** - Coordinates flywheel spin-up, hood angle, and conveyor feeding.
- **TurretCalibrationCommand** - Interactive calibration in test mode; records distance/angle data.

### Turret calibration pipeline
CSV calibration data → `GenerateLookupTables.java` (build-time task) → `generated/TurretLookupTables.java`. The `compileJava` task depends on `generateTurretLookupTables`, so lookup tables are always current. See `docs/turret-calibration-guide.md` for the operator workflow.

### Configuration
All tunable parameters (PID gains, gear ratios, current limits, CAN IDs) live in `Constants.java` organized by nested static classes per subsystem.

## Code Conventions

- **WPILib command framework v2**: subsystems extend `SubsystemBase`, commands extend `Command` with `addRequirements()`.
- **CTRE Phoenix6 motor pattern**: TalonFX configuration in constructor, control requests (VelocityTorqueCurrentFOC, MotionMagicTorqueCurrentFOC), status signal caching.
- **Constants naming**: `UPPER_SNAKE_CASE` for `public static final` fields in `Constants.java`.
- **Alliance-aware**: Vision and aiming logic switches targets based on `DriverStation.getAlliance()`.
- **Signal update frequency**: Status signals use `Constants.SIGNAL_UPDATE_FREQUENCY_HZ` (250 Hz).

## Vendor Libraries

Phoenix6 (CTRE motors/sensors), PathplannerLib (autonomous paths), REVLib (REV hardware), photonlib (vision), questnavlib (Quest 3 tracking), ChoreoLib (trajectories). Dependency JSONs are in `vendordeps/`.
