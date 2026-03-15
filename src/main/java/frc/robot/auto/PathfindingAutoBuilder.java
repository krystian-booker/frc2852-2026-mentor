package frc.robot.auto;

import com.google.gson.JsonArray;
import com.google.gson.JsonObject;
import com.google.gson.JsonParser;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants;

import java.io.BufferedReader;
import java.io.File;
import java.io.FileReader;
import java.io.IOException;
import java.util.function.Supplier;

/**
 * Builds auto commands that pathfind to the starting pose of the first path before running the full
 * auto. This allows the robot to start anywhere within a few feet of the expected position and
 * dynamically drive to the correct starting location.
 */
public final class PathfindingAutoBuilder {

    private PathfindingAutoBuilder() {
    }

    /**
     * Builds a command that pathfinds to the start of the auto's first path, then runs the full
     * auto.
     *
     * @param autoName the name of the auto (without .auto extension)
     * @return command sequence: pathfind to start → full auto
     */
    /**
     * Builds a command that simply runs the PathPlanner auto without any initial pathfinding.
     *
     * @param autoName the name of the auto (without .auto extension)
     * @return the auto command
     */
    public static Command buildAuto(String autoName) {
        return new PathPlannerAuto(autoName);
    }

    /**
     * Builds a command that pathfinds to the start of the auto's first path, then runs the full
     * auto.
     *
     * @param autoName the name of the auto (without .auto extension)
     * @return command sequence: pathfind to start → full auto
     */
    public static Command buildAutoWithPathfinding(String autoName) {
        return buildAutoWithPathfinding(autoName, () -> new Pose2d());
    }

    /**
     * Builds a command that pathfinds to the start of the auto's first path, then runs the full
     * auto. Accepts a pose supplier for diagnostic logging.
     *
     * @param autoName     the name of the auto (without .auto extension)
     * @param poseSupplier supplies the robot's current pose for logging
     * @return command sequence: pathfind to start → full auto
     */
    public static Command buildAutoWithPathfinding(String autoName, Supplier<Pose2d> poseSupplier) {
        try {
            String firstPathName = getFirstPathName(autoName);
            if (firstPathName == null) {
                System.out.println("[PathfindingAuto] No path found in auto '" + autoName + "', running without pathfinding");
                return new PathPlannerAuto(autoName);
            }

            PathPlannerPath firstPath = PathPlannerPath.fromPathFile(firstPathName);
            Pose2d startPose = firstPath.getStartingHolonomicPose()
                    .orElse(firstPath.getStartingDifferentialPose());

            System.out.println("[PathfindingAuto] Target start pose: " + startPose + " for auto '" + autoName + "'");

            PathConstraints constraints = new PathConstraints(
                    Constants.AutoConstants.PATHFINDING_MAX_VELOCITY,
                    Constants.AutoConstants.PATHFINDING_MAX_ACCELERATION,
                    Constants.AutoConstants.PATHFINDING_MAX_ANGULAR_VELOCITY,
                    Constants.AutoConstants.PATHFINDING_MAX_ANGULAR_ACCELERATION);

            Command pathfindToStart = AutoBuilder.pathfindToPoseFlipped(startPose, constraints)
                    .withTimeout(Constants.AutoConstants.PATHFINDING_TIMEOUT_SECONDS);

            // Schedule PathPlannerAuto independently after pathfinding completes.
            // PathPlannerAuto must NOT be composed inside a command group — it manages
            // its own internal command scheduling and doesn't work correctly when nested.
            return Commands.sequence(
                    Commands.runOnce(() -> System.out.println(
                            "[PathfindingAuto] Robot pose: " + poseSupplier.get() + " | Pathfinding to: " + startPose)),
                    pathfindToStart,
                    Commands.runOnce(() -> System.out.println(
                            "[PathfindingAuto] Pathfinding done, scheduling auto '" + autoName + "'"))
            ).finallyDo(interrupted -> {
                if (!interrupted) {
                    System.out.println("[PathfindingAuto] Scheduling PathPlannerAuto '" + autoName + "'");
                    new PathPlannerAuto(autoName).schedule();
                } else {
                    System.out.println("[PathfindingAuto] Auto cancelled, not scheduling PathPlannerAuto");
                }
            });
        } catch (Exception e) {
            DriverStation.reportError(
                    "Failed to build pathfinding auto for '" + autoName + "': " + e.getMessage(),
                    e.getStackTrace());
            return new PathPlannerAuto(autoName);
        }
    }

    /**
     * Parses the auto JSON file to find the first path command's pathName. Walks the command tree
     * recursively to handle nested groups (sequential, parallel, etc.).
     *
     * @param autoName the auto name (without .auto extension)
     * @return the first path name found, or null if no path commands exist
     */
    private static String getFirstPathName(String autoName) {
        File autoFile = new File(Filesystem.getDeployDirectory(),
                "pathplanner/autos/" + autoName + ".auto");
        if (!autoFile.exists()) {
            DriverStation.reportError("Auto file not found: " + autoFile.getAbsolutePath(), false);
            return null;
        }

        try (BufferedReader reader = new BufferedReader(new FileReader(autoFile))) {
            StringBuilder sb = new StringBuilder();
            String line;
            while ((line = reader.readLine()) != null) {
                sb.append(line);
            }
            JsonObject autoJson = JsonParser.parseString(sb.toString()).getAsJsonObject();
            JsonObject command = autoJson.getAsJsonObject("command");
            return findFirstPathName(command);
        } catch (IOException e) {
            DriverStation.reportError(
                    "Failed to read auto file '" + autoName + "': " + e.getMessage(), false);
            return null;
        }
    }

    /**
     * Recursively searches a PathPlanner command JSON object for the first path command.
     *
     * @param command the command JSON object
     * @return the path name if found, or null
     */
    private static String findFirstPathName(JsonObject command) {
        String type = command.get("type").getAsString();
        JsonObject data = command.getAsJsonObject("data");

        if ("path".equals(type)) {
            return data.get("pathName").getAsString();
        }

        // For group types (sequential, parallel, race, deadline), search children
        if (data.has("commands")) {
            JsonArray commands = data.getAsJsonArray("commands");
            for (int i = 0; i < commands.size(); i++) {
                String result = findFirstPathName(commands.get(i).getAsJsonObject());
                if (result != null) {
                    return result;
                }
            }
        }

        return null;
    }

}
