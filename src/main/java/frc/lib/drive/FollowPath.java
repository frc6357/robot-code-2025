package frc.lib.drive;

import static frc.robot.Konstants.AutoConstants.kDefaultPathfindingConstraints;

import java.io.FileNotFoundException;
import java.io.IOException;

import org.json.simple.parser.ParseException;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.util.FileVersionException;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

public class FollowPath {


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
     * Follows a pre-generated PathPlanner path, first generating a path from the robot's
     * current position to the starting point of the path.
     * @param pathName The name of the path file to follow
     * @return Pathfinding and following command
     */
    public static Command PathfindThenFollowPathCommand(String pathName) {
        return PathfindThenFollowPathCommand(pathName, kDefaultPathfindingConstraints);
    }

    /**
     * Follows a pre-generated PathPlanner path, first generating a path from the robot's
     * current position to the starting point of the path.
     * @param pathName The name of the path file to follow
     * @param constraints The path constraints to use when generating the path to the start
     * @return Pathfinding and following command
     */
    public static Command PathfindThenFollowPathCommand(String pathName, PathConstraints constraints) {
        try{
            return AutoBuilder.pathfindThenFollowPath(PathPlannerPath.fromPathFile(pathName), constraints);
        }
        catch (Exception e) {
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
}