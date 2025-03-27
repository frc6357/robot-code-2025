package frc.robot.commands.DriveToReef;

import java.util.function.Supplier;

import edu.wpi.first.math.controller.ProfiledPIDController;
// import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
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


    public RotateToReef(MultiLimelightCommandConfig c, SKSwerve m_swerve) {
        this.config = c;
        this.m_swerve = m_swerve;


        Constraints constraints = new Constraints(config.maxVelocity, config.maxAcceleration);
        rotPID = new ProfiledPIDController(config.kp, config.ki, config.kd, constraints);

        rotPID.enableContinuousInput(-Math.PI, Math.PI);
        rotPID.setTolerance(0); // TODO: Change back to config.tolerance

        this.currentHeading = () -> (m_swerve.getRobotRotation().getRadians());

        outputting = true;
    }

    public void initialize(Pose2d targetPose) {
        this.targetHeading = () -> (targetPose.getRotation().getRadians());

        rotPID.setGoal(targetHeading.get());
        rotPID.reset(currentHeading.get(), m_swerve.getVelocity(false).omegaRadiansPerSecond); // Position, velocity
    }

    public double getOutput() {

        outputting = (rotPID.getPositionError() > 0); //TODO: Change 0 back to config.error
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
