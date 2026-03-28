package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.Hood;

/**
 * Automated test sequence that exercises the hood through its full range for diagnostic data
 * collection. Bind to a button in test mode — the hood diagnostic logger captures all data while
 * this runs.
 *
 * <p>Sequence: zeros the hood via stall-current homing, then slow steps (2s holds) covering
 * small/large moves in both directions, then rapid transitions (1s holds). Total runtime ~25
 * seconds.
 */
public class HoodTestSequenceCommand {

    private HoodTestSequenceCommand() {
    }

    public static Command create(Hood hood) {
        return Commands.sequence(
                // Zero the hood first — drives to reverse hard stop, detects stall, sets encoder to 0
                hood.zeroHoodCommand(),
                // Slow steps with 2s holds
                stepAndHold(hood, 0.0, 2.0),
                stepAndHold(hood, 5.0, 2.0),
                stepAndHold(hood, 15.0, 2.0),
                stepAndHold(hood, 10.0, 2.0),
                stepAndHold(hood, 25.0, 2.0),
                stepAndHold(hood, 12.0, 2.0),
                stepAndHold(hood, 0.0, 2.0),
                // Rapid transitions with 1s holds
                stepAndHold(hood, 20.0, 1.0),
                stepAndHold(hood, 5.0, 1.0),
                stepAndHold(hood, 25.0, 1.0),
                stepAndHold(hood, 0.0, 1.0))
                .finallyDo(() -> hood.setPosition(0.0))
                .withName("HoodTestSequence");
    }

    private static Command stepAndHold(Hood hood, double degrees, double holdSeconds) {
        return Commands.sequence(
                hood.runOnce(() -> hood.setPosition(degrees)),
                Commands.waitSeconds(holdSeconds));
    }
}
