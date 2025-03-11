package frc.robot.commands.DriveToReef;

import java.util.function.Supplier;

import edu.wpi.first.math.controller.ProfiledPIDController;
// import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import frc.robot.commands.DriveCommand;
import frc.robot.subsystems.SKSwerve;
import frc.robot.subsystems.vision.SK25Vision.MultiLimelightCommandConfig;

/**
 * This is to have a very similar layout to a command with the "execute"
 * body mostly contained within the getOutput() method since that's where the 
 * PID controller updates. This is intended to be used within a command and not as a 
 * standalone class.
 * @see TranslateToReef
 */
public class RotateToReef {
    private MultiLimelightCommandConfig config;

    private SKSwerve m_swerve;
    
    private ProfiledPIDController rotPID;

    private Supplier<Double> targetHeading;

    private Supplier<Double> currentHeading;

    // private SlewRateLimiter slewFilter;

    private boolean outputting;


    public RotateToReef(MultiLimelightCommandConfig config, SKSwerve m_swerve) {
        this.config = config;
        this.m_swerve = m_swerve;

        //TODO: Maybe add slew rate limiter based on robot performance?

        Constraints constraints = new Constraints(config.maxVelocity, config.maxAcceleration);
        rotPID = new ProfiledPIDController(config.kp, config.ki, config.kd, constraints);

        rotPID.enableContinuousInput(-Math.PI, Math.PI);
        rotPID.setTolerance(config.tolerance);

        this.currentHeading = () -> (m_swerve.getRobotPose().getRotation().getRadians());

        outputting = true;
    }

    public void initialize(Pose2d targetPose) {
        this.targetHeading = () -> (targetPose.getRotation().getRadians());

        rotPID.reset(currentHeading.get(), m_swerve.getVelocity(true).omegaRadiansPerSecond); // Position, velocity
    }

    public double getOutput() {
        outputting = (rotPID.getPositionError() > config.error);
        if(!outputting) {
            return 0;
        }
        else {
            return rotPID.calculate(currentHeading.get(), targetHeading.get());
        }
    }

    public void end() {
        outputting = false;
        rotPID.setGoal(currentHeading.get());
    }

    public boolean isFinished() {
        return !outputting;
    }
}
