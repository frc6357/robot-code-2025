package frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.subsystems.SKSwerve;
import frc.robot.subsystems.vision.SK25Vision.CommandConfig;
import frc.robot.utils.vision.Limelight;
import frc.robot.commands.DriveCommand;

public class DriveToPoseCommand extends Command {
    private static SKSwerve m_swerve = RobotContainer.m_swerve;

    private CommandConfig translateConfig;
    private CommandConfig rotateConfig;
    private CommandConfig driveToPoseConfig;

    private static Limelight limelight;
    DriveCommand driveCommand;

    private Pose2d targetPose;
    private double targetPoseX;
    private double targetPoseY;

    private SlewRateLimiter slewFilter;
    private PIDController xPID;
    private PIDController yPID;
    private PIDController rotPID;

    private Supplier<Pose2d> robotPose;
    private Supplier<Double> robotPoseX;
    private Supplier<Double> robotPoseY;

    private static double outputX;
    private static double outputY;

    // private Supplier<Double> rotRateSupplier; // Supplies drivetrain's current rotation rate in order to 
    //                                           // move the robot's translation while maintaining rotational input

    public DriveToPoseCommand(CommandConfig translateConfig, CommandConfig rotateConfig, Pose2d targetPose, SKSwerve m_swerve) {
        
    }
}
