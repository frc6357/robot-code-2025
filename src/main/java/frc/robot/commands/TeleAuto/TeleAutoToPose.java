// package frc.robot.commands.TeleAuto;

// import java.util.function.BiFunction;

// import com.pathplanner.lib.auto.AutoBuilder;
// import com.pathplanner.lib.auto.AutoBuilder.TriFunction;
// import com.pathplanner.lib.commands.PathfindThenFollowPath;
// import com.pathplanner.lib.path.PathConstraints;
// import com.pathplanner.lib.path.PathPlannerPath;

// import frc.robot.subsystems.SKSwerve;

// import edu.wpi.first.math.geometry.Pose2d;
// import edu.wpi.first.wpilibj2.command.Command;

// public class TeleAutoToPose extends Command {
//     private Pose2d targetPose;
//     private SKSwerve m_swerve;
//     private PathConstraints constraints;
//     private double goalEndVel;

//     private 

//     private TriFunction<Pose2d, PathConstraints, Double, Command> pathfindToPoseBuilder;
//     private BiFunction<PathPlannerPath, PathConstraints, Command> pathfindThenFollowBuilder;


//     /**
//      * This creates a path on the fly for the robot to follow. If you wish to 
//      * move the robot for a precise alignment, consider using tag-relative vision
//      * detections and measurements in order to reduce inconsistencies. Running 
//      * autonomous paths requires precise odometry, so make sure the path is designed
//      * such that it has a low chance of causing a scoring attempt to go wrong.
//      * @param m_swerve The swerve drivetrain for the command to use
//      * @param targetPose The pose for the path generator to target
//      * @param constraints The PathConstraints for the robot to follow when generating and running
//      * @param goalEndVel The goal velocity of the robot at the end of the path
//      */
//     public PathfindThenFollowPath TeleAutoToPose(SKSwerve m_swerve, Pose2d targetPose, PathConstraints constraints, double goalEndVel) {
//         this.m_swerve = m_swerve;
//         this.targetPose = targetPose;
//         this.constraints = constraints;
//         this.goalEndVel = goalEndVel;

//         return AutoBuilder.pathfindThenFollowPath(null, constraints)
//     }
// }