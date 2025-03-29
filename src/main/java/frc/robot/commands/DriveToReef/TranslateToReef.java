package frc.robot.commands.DriveToReef;

import java.util.function.Supplier;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.subsystems.SKSwerve;
import frc.robot.subsystems.vision.SK25Vision.CommandConfig;

/**
 * This is to have a very similar layout to a command with the "execute"
 * body mostly contained within the getOutput() method since that's where the 
 * PID controller updates. This is intended to be used within a command and not as a 
 * standalone class.
 * @see RotateToReef
 */
public class TranslateToReef{
    private CommandConfig config;
    
    private SKSwerve m_swerve;

    private ProfiledPIDController xPID;
    private ProfiledPIDController yPID;

    private double targetX;
    private double targetY;

    private Supplier<Double> currentX;
    private Supplier<Double> currentY;

    private boolean outputtingX;
    private boolean outputtingY;
    
    public TranslateToReef(CommandConfig c, SKSwerve m_swerve) {
        this.config = c;
        this.m_swerve = m_swerve;

        Constraints constraints = new Constraints(config.maxVelocity, config.maxAcceleration);
        xPID = new ProfiledPIDController(config.kp, config.ki, config.kd, constraints);
        xPID.setTolerance(config.tolerance);

        yPID = new ProfiledPIDController(config.kp, config.ki, config.kd, constraints);
        yPID.setTolerance(config.tolerance);


        currentX = () -> (m_swerve.getRobotPose().getX());
        currentY = () -> (m_swerve.getRobotPose().getY());

        outputtingX = true;
        outputtingY = true;
    }

    public void initialize(Pose2d targetPose) {
        this.targetX = targetPose.getX();
        this.targetY = targetPose.getY();
        reset();

        xPID.setGoal(targetX);
        yPID.setGoal(targetY);
    }

    public double getXGoal() {
        return xPID.getGoal().position;
    }
    public double getYGoal() {
        return yPID.getGoal().position;
    }

    public double getXOutput() {
        outputtingX = !(xPID.atGoal());

        if(!outputtingX) {
            return 0;
        }
        else {
            if(!(DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red)) {
                return xPID.calculate(currentX.get(), targetX);
            }
            return xPID.calculate(currentX.get(), targetX) * -1;
        }
    }

    public double getYOutput() {
        outputtingY = !(yPID.atGoal());
        
        if(!outputtingY) {
            return 0;
        }
        else {
            if(!(DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red)) {
                return yPID.calculate(currentY.get(), targetY);
            }
            return yPID.calculate(currentY.get(), targetY) * -1;
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
