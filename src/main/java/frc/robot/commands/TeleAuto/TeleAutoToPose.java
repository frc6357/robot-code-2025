package frc.robot.commands.TeleAuto;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathConstraints;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;

public class TeleAutoToPose {

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
    public Command newTeleAutoToPose(Pose2d targetPose, PathConstraints constraints, double goalEndVel) {
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
    public Command newTeleAutoToPose(Pose2d targetPose, PathConstraints constraints) {
        return AutoBuilder.pathfindToPose(targetPose, constraints);
    }
}