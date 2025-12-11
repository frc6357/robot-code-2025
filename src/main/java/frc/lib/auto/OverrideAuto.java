package frc.lib.auto;

import static frc.robot.Konstants.AutoConstants.kDefaultPathfindingConstraints;

import java.io.FileNotFoundException;
import java.io.IOException;
import java.text.ParseException;
import java.util.function.Supplier;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.events.EventScheduler;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.trajectory.PathPlannerTrajectory;
import com.pathplanner.lib.util.FileVersionException;
import com.pathplanner.lib.util.PathPlannerLogging;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.DeferredCommand;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

public class OverrideAuto {

    public static Command InterjectAutoAndRecover(Supplier<Command> selectedAutoCommand, Command toRun) {
        PathPlannerAuto currentPathPlannerAuto = null;

        // If no path is active
        if(PathPlannerAuto.currentPathName.equals("")) {
            System.out.println("No path currently active to reference");
            return Commands.none();
        }

        try {
            currentPathPlannerAuto = (PathPlannerAuto)selectedAutoCommand.get();
        }
        catch(ClassCastException e) {
            System.out.println("Selected auto command is not a PathPlannerAuto command. Cannot interject.");
            return Commands.none();
        }

        // If the current auto is running
        if(currentPathPlannerAuto.isRunning().getAsBoolean()) {
            PathPlannerPath currentPath;

            // Try to load the currently used path by the PathPlannerAuto system
            try {
                currentPath = PathPlannerPath.fromPathFile(PathPlannerAuto.currentPathName);
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
                
                return Commands.none();
            }

            PathPlannerTrajectory trajectory = null;
            try {
                trajectory = currentPath.getIdealTrajectory(RobotConfig.fromGUISettings()).orElseThrow();
            }
            catch(Exception ex) {
                DriverStation.reportError("Unable to load RobotConfig from PP GUI", true);
            }

            if(trajectory == null) {
                System.out.println("Next trajectory in path recovery returned null");
                return Commands.none();
            }

            return Commands.sequence(
                toRun,
                Pathfinder.PathfindToPoseCommand(
                    trajectory.getInitialState().pose, 
                    kDefaultPathfindingConstraints, 
                    trajectory.getInitialState().linearVelocity)
            );
        }

        return Commands.none();
    }

    // public static Command createCommand(Command overrideAutoCommand, Pose2d initialPose, double goalEndVelocity) {
    //     return Commands.sequence(
    //         overrideAutoCommand,
    //         Pathfinder.PathfindToPoseCommand(initialPose, kDefaultPathfindingConstraints, goalEndVelocity)
    //     );
    // }
        
}
