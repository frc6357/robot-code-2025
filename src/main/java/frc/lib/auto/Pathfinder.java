package frc.lib.auto;

import java.io.FileNotFoundException;
import java.io.IOException;
import java.text.ParseException;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.util.FileVersionException;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

public class Pathfinder {

    /**
     * This creates a path on the fly for the robot to follow. If you wish to 
     * move the robot for a precise alignment, consider using tag-relative vision
     * detections and measurements in order to reduce inconsistencies. Running 
     * autonomous paths requires precise odometry, so make sure the path is designed
     * such that it has a low chance of causing a scoring attempt to go wrong.
     * @param targetPose The pose for the path generator to target
     * @param constraints The PathConstraints for the robot to follow when generating and running
     * @param goalEndVel The goal velocity of the robot at the end of the path
     */
    public static Command PathfindToPoseCommand(Pose2d targetPose, PathConstraints constraints, double goalEndVel) {
        return AutoBuilder.pathfindToPose(targetPose, constraints, goalEndVel);
    }

    /**
     * This creates a path on the fly for the robot to follow. If you wish to 
     * move the robot for a precise alignment, consider using tag-relative vision
     * detections and measurements in order to reduce inconsistencies. Running 
     * autonomous paths requires precise odometry, so make sure the path is designed
     * such that it has a low chance of causing a scoring attempt to go wrong.
     * @param targetPose The pose for the path generator to target
     * @param constraints The PathConstraints for the robot to follow when generating and running
     * @param goalEndVel = 0.0 m/s
     */
    public static Command PathfindToPoseCommand(Pose2d targetPose, PathConstraints constraints) {
        return AutoBuilder.pathfindToPose(targetPose, constraints);
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

    public static Command PathFindToStartOfPathCommand(String pathName, PathConstraints constraints) {
        PathPlannerPath path = null;

        try {
            path = PathPlannerPath.fromPathFile(pathName);
        }
        catch (Exception e) {
            if(e instanceof IOException) {
                System.out.println("Auto file not found");
            }
            else if(e instanceof ParseException) {
                System.out.println("JSON could not be parsed");
            }
        }

        if(path == null) {
            return Commands.none();
        }

        // Pathfind to the starting pose and use the ideal starting state to determine
        // the goal velocity and rotation
        return Pathfinder.PathfindToPoseCommand(
            new Pose2d(
                path.getStartingHolonomicPose().orElseThrow().getTranslation(),
                path.getIdealStartingState().rotation()
            ), 
            constraints, 
            path.getIdealStartingState().velocityMPS()
        );
    }

    /**
     * Creates a command that pathfinds to the first position of an auto path sequence
     * @param autoName The name of the auto sequence
     * @return The command
     */
    public static Command PathfindToStartOfAutoCommand(String autoName, PathConstraints constraints) {
        PathPlannerPath firstPath = null;

        try {
            firstPath = PathPlannerAuto.getPathGroupFromAutoFile(autoName).get(0);
        }
        catch (Exception e) {
            if(e instanceof IOException) {
                System.out.println("Auto file not found");
            }
            else if(e instanceof ParseException) {
                System.out.println("JSON could not be parsed");
            }
        }

        if(firstPath == null) {
            return Commands.none();
        }

        // Pathfind to the starting pose and use the ideal starting state to determine
        // the goal velocity and rotation
        return Pathfinder.PathfindToPoseCommand(
            new Pose2d(
                firstPath.getStartingHolonomicPose().orElseThrow().getTranslation(),
                firstPath.getIdealStartingState().rotation()
            ), 
            constraints, 
            firstPath.getIdealStartingState().velocityMPS()
        );
    }
}