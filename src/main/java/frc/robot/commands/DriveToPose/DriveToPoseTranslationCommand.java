package frc.robot.commands.DriveToPose;

import java.util.function.Supplier;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.subsystems.SKSwerve;
import frc.robot.subsystems.vision.SK25Vision.CommandConfig;
import frc.robot.utils.vision.Limelight;
import frc.robot.commands.DriveCommand;

public class DriveToPoseTranslationCommand extends Command {
    private static SKSwerve m_swerve = RobotContainer.m_swerve;

    private CommandConfig config;

    private static Limelight limelight;
    DriveCommand driveCommand;

    private Pose2d targetPose;
    private double targetPoseX;
    private double targetPoseY;

    private PIDController xPID;
    private PIDController yPID;

    private Supplier<Pose2d> robotPose;
    private Supplier<Double> robotPoseX;
    private Supplier<Double> robotPoseY;

    private static double outputX;
    private static double outputY;

    private Supplier<Double> rotRateSupplier; // Supplies drivetrain's current rotation rate in order to 
                                              // move the robot's translation while maintaining rotational input

    public DriveToPoseTranslationCommand(CommandConfig config, Pose2d targetPose, Supplier<Double>rotRateSupplier, SKSwerve m_swerve) {
        xPID = new PIDController(config.kp, 0, 0);
        xPID.setTolerance(config.tolerance);
        yPID = new PIDController(config.kp, 0, 0);
        yPID.setTolerance(config.tolerance);

        this.rotRateSupplier = rotRateSupplier;

        robotPose = () -> (m_swerve.getRobotPose());
        robotPoseX = () -> (m_swerve.getRobotPose().getX());
        this.targetPoseX = targetPose.getX();
        robotPoseY = () -> (m_swerve.getRobotPose().getY());
        this.targetPoseY = targetPose.getY();

        this.targetPose = targetPose;

        config = this.config;
        limelight = config.limelight;

        addRequirements(m_swerve);

        driveCommand = 
                new DriveCommand(
                    () -> (getOutputX()), 
                    () -> (getOutputY()), 
                    this.rotRateSupplier, 
                    () -> (false));
    }
    
    private void setOutputX(double out) {
        outputX = out;
        if(Math.abs(outputX) > 1) {
            outputX = 1 * Math.signum(outputX);
        }

        outputX *= config.maxOutput; // maxOutput is the maximum allowed drivetrain speed for the specific command
    }

    private void setOutputY(double out) {
        outputY = out;
        if(Math.abs(outputY) > 1) {
            outputY = 1 * Math.signum(outputY);
        }

        outputY *= config.maxOutput;
    }

    private void resetOutputs() {
        outputX = 0;
        outputY = 0;
    }

    private double getOutputX() {
        return outputX;
    }
    private double getOutputY() {
        return outputY;
    }

    private boolean atXTarget() {
        return xPID.atSetpoint();
    }
    private boolean atYTarget() {
        return yPID.atSetpoint();
    }
    private boolean atTarget() {
        return (atXTarget() && atYTarget());
    }


    @Override
    public void initialize() {
        xPID.reset();
        resetOutputs();

        // driveCommand.initialize();
        limelight.setLimelightPipeline(config.pipelineIndex);
    }

    @Override
    public void execute() {
        if(!atXTarget()) {
            setOutputX(xPID.calculate(robotPoseX.get(), targetPoseX));
        }
        else {
            setOutputX(0);
        }
        if(!atYTarget()) {
            setOutputY(yPID.calculate(robotPoseY.get(), targetPoseY));
        }
        else {
            setOutputY(0);
        }

        driveCommand.run();
    }

    @Override
    public void end(boolean interrupted) {}

    @Override
    public boolean isFinished() {
        return atTarget();
    }
}
