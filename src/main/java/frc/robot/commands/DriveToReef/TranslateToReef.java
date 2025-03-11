package frc.robot.commands.DriveToReef;

import static frc.robot.Konstants.VisionConstants.kAprilTagPipeline;

import java.util.function.Supplier;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.commands.DriveCommand;
import frc.robot.subsystems.SKSwerve;
import frc.robot.subsystems.vision.SK25Vision.CommandConfig;
import frc.robot.utils.vision.Limelight;

/**
 * This is to have a very similar layout to a command with the "execute"
 * body mostly contained within the getOutput() method since that's where the 
 * PID controller updates. This is intended to be used within a command and not as a 
 * standalone class.
 * @see RotateToReef
 */
public class TranslateToReef{
    private CommandConfig config;
    private Limelight[] limelights;
    DriveCommand driveCommand;
    
    private SKSwerve m_swerve;

    private ProfiledPIDController xPID;
    private ProfiledPIDController yPID;

    private double targetX;
    private double targetY;

    private double xOut;
    private double yOut;

    private Supplier<Pose2d> currentPose;
    private Supplier<Double> currentX;
    private Supplier<Double> currentY;

    private boolean outputtingX;
    private boolean outputtingY;
    
    public TranslateToReef(CommandConfig config, Pose2d targetPose, SKSwerve m_swerve) {
        this.config = config;
        this.m_swerve = m_swerve;

        Constraints constraints = new Constraints(config.maxVelocity, config.maxAcceleration);
        xPID = new ProfiledPIDController(config.kp, config.ki, config.kd, constraints);
        xPID.setTolerance(config.tolerance);

        yPID = new ProfiledPIDController(config.kp, config.ki, config.kd, constraints);
        yPID.setTolerance(config.tolerance);

        this.targetX = targetPose.getX();
        this.targetY = targetPose.getY();

        currentX = () -> (m_swerve.getRobotPose().getX());
        currentY = () -> (m_swerve.getRobotPose().getY());

        outputtingX = true;
        outputtingY = true;
    }

    public void initialize() {
        reset();
    }

    public double getXOutput() {
        outputtingX = !(xPID.atGoal());

        if(!outputtingX) {
            return 0;
        }
        else {
            return xPID.calculate(currentX.get(), targetX);
        }
    }

    public double getYOutput() {
        outputtingY = !(yPID.atGoal());
        
        if(!outputtingY) {
            return 0;
        }
        else {
            return yPID.calculate(currentY.get(), targetY);
        }
    }

    public void end() {
        outputtingX = false;
        outputtingY = false;

        xPID.setGoal(currentX.get());
        yPID.setGoal(currentY.get());
    }

    public boolean isFinished() {
        return(!outputtingX && !outputtingY);
    }

    public void reset() {
        xPID.reset(currentX.get(), getVelocities().vxMetersPerSecond);
        yPID.reset(currentY.get(), getVelocities().vyMetersPerSecond);
    }

    private ChassisSpeeds getVelocities() { // Always field-centric
        return m_swerve.getVelocity(true);
    }
}
