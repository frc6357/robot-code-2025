package frc.lib.auto;

import java.io.FileNotFoundException;
import java.io.IOException;
import java.util.ArrayList;
import java.util.List;

import org.json.simple.parser.ParseException;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.util.FileVersionException;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

public class Pathfollower {
    /**
     * Follows a pre-generated PathPlanner path, assuming the robot's position is ideally
     * very close to the path's starting point.
     * @param pathName The name of the path file to follow
     * @return Path following command
     */
    public static Command FollowPathCommand(String pathName) {
        try {
            return AutoBuilder.followPath(PathPlannerPath.fromPathFile(pathName));
        } catch (Exception e) {
            if(e instanceof IOException) {
                System.out.println("Path not found");
            }
            else if(e instanceof ParseException) {
                System.out.println("JSON could not be parsed");
            }
            else if(e instanceof FileNotFoundException) {
                System.out.println("Path file not found");
            }
            else if(e instanceof FileVersionException) {
                System.out.println("Path file version is not compatible with this version of PathPlanner");
            }
            else {
                System.out.println("Unknown error occurred while loading path");
            }
        }
        return Commands.none();
    }

    /**
     * Follows a PathPlanner path sequence loaded from an auto file.
     * @param autoName The name of the auto file to load
     * @return Path sequence following command
     */
    public static Command FollowAutoSequenceCommand(String autoName) {
        List<PathPlannerPath> pathSequence = generatePathSequence(autoName);

        if(pathSequence != null) {
            List<Command> followPathCommands = generateFollowPathSequence(pathSequence);
            return Commands.sequence(followPathCommands.toArray(new Command[0]));
        }

        return Commands.none();
    }

    private static List<PathPlannerPath> generatePathSequence(String autoName) {
        try {
            List<PathPlannerPath> extractedPathSequence = PathPlannerAuto.getPathGroupFromAutoFile(autoName);
            return extractedPathSequence;
        }
        catch (Exception e) {
            if(e instanceof IOException) {
                System.out.println("Auto file not found");
            }
            else if(e instanceof ParseException) {
                System.out.println("JSON could not be parsed");
            }
        }

        return null;
    }

    private static List<Command> generateFollowPathSequence(List<PathPlannerPath> pathSequence) {
        List<Command> followPathCommands = new ArrayList<>();

        for(PathPlannerPath path : pathSequence) {
            followPathCommands.add(AutoBuilder.followPath(path));
        }
        // followPathCommands.set(0, AutoBuilder.pathfindThenFollowPath(pathSequence.get(0), kDefaultPathfindingConstraints));

        return followPathCommands;
    }
}