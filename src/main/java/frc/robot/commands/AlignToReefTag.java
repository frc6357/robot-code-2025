package frc.robot.commands;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.SKSwerve;
import frc.robot.subsystems.vision.SK25Vision;
import frc.robot.subsystems.vision.SK25Vision.MultiLimelightCommandConfig;
import frc.robot.utils.vision.Limelight;
import frc.robot.utils.vision.LimelightHelpers.RawFiducial;
import static frc.robot.Konstants.VisionConstants.AlignmentConstants.LeftLimelight;
import static frc.robot.Konstants.VisionConstants.AlignmentConstants.RightLimelight;
import static frc.robot.Ports.DriverPorts.kDriver;
import static frc.robot.Konstants.VisionConstants.AlignmentConstants;
import static frc.robot.Konstants.VisionConstants.limelightAlpha;
import static frc.robot.Konstants.VisionConstants.limelightBeta;


public class AlignToReefTag extends Command {
    private ProfiledPIDController xPID, yPID, rotPID;
    private SKSwerve m_swerve;
    private MultiLimelightCommandConfig xyConfig;
    private MultiLimelightCommandConfig rotConfig;
    private Constraints xyConstraints, rotConstraints;
    private RawFiducial closestTag;
    private DriveCommand driveCommand;
    private SK25Vision m_vision;
    private Limelight[] limelights;
    private Limelight targetLimelight;
    private Target target;
    private boolean valid;

    private double xOut;
    private double yOut;
    private double rotOut;

    static enum Target {
        CENTER,
        LEFT,
        RIGHT,
        BACK
    }

    public AlignToReefTag(
            Target target, 
            MultiLimelightCommandConfig xyConfig,
            MultiLimelightCommandConfig rotConfig,
            SK25Vision m_vision,
            SKSwerve m_swerve) {

        this.target = target;
        this.xyConfig = xyConfig;
        this.rotConfig = rotConfig;
        this.m_vision = m_vision;
        this.m_swerve = m_swerve;
        this.limelights = xyConfig.limelights;

        rotConstraints = new Constraints(rotConfig.maxVelocity, rotConfig.maxAcceleration);
        xyConstraints = new Constraints(xyConfig.maxVelocity, xyConfig.maxAcceleration);
        
        rotPID = new ProfiledPIDController(
            rotConfig.kp,
            rotConfig.ki,
            rotConfig.kd,
            rotConstraints);
        xPID = new ProfiledPIDController(
            xyConfig.kp,
            xyConfig.ki,
            xyConfig.kd,
            xyConstraints);
        yPID = new ProfiledPIDController(
            xyConfig.kp,
            xyConfig.ki,
            xyConfig.kd,
            xyConstraints);
        
        this.driveCommand = new DriveCommand(
            () -> getXOutput(), 
            () -> getYOutput(), 
            () -> getRotOutput(), 
            () -> false);

        addRequirements(m_swerve);
    }

    @Override
    public void initialize() {
        resetAll();
        setTarget();
    }

    @Override
    public void execute() {
        if(valid) {
            calculateSpeeds();
            
            m_vision.isDriving = true;

            if(DriverStation.isTeleopEnabled()) {
                kDriver.setRumble(RumbleType.kBothRumble, 0.5);
            }

            driveCommand.run();
        }
        else {
            kDriver.setRumble(RumbleType.kBothRumble, 0.0);
        }
        SmartDashboard.putNumber("Align/xPIDOutput", xOut);
        SmartDashboard.putNumber("Align/yPIDOutput", yOut);
        SmartDashboard.putNumber("Align/rotPIDOutput", rotOut);
    }

    @Override
    public void end(boolean isInterrupted) {
        kDriver.setRumble(RumbleType.kBothRumble, 0.0);
        m_vision.reefDriveTarget = "OFF";
        m_vision.isDriving = false;
        
        shutdownAll();
    }

    @Override
    public boolean isFinished() {
        if(valid) {
            return xPID.atSetpoint() && yPID.atSetpoint() && rotPID.atSetpoint();
        }
        return !valid;
    }

    private void calculateSpeeds() {
        xOut = xyConfig.maxVelocity * xPID.calculate(targetLimelight.getCameraPoseTS3d().toPose2d().getX());
        yOut = xyConfig.maxVelocity * -yPID.calculate(targetLimelight.getCameraPoseTS3d().toPose2d().getY());
        rotOut = rotConfig.maxVelocity * -rotPID.calculate(targetLimelight.getCameraPoseTS3d().toPose2d().getRotation().getDegrees());
    }

    private Limelight findGoodLimelight() {
        for(Limelight ll : limelights) {
            if(m_vision.reefTargetClose(ll)) {
                return ll;
            }
        }
        return null;
    }

    private boolean setTargetLimelight(Target t) {
        switch(t) {
            case LEFT:
                if(!m_vision.reefTargetClose(limelights[0])) {
                    break;
                }
                targetLimelight = limelights[0];
                break;
            case RIGHT:
                if(!m_vision.reefTargetClose(limelights[1])) {
                    break;
                }
                targetLimelight = limelights[1];
                break;
            case CENTER:
                targetLimelight = findGoodLimelight();
                break;
            case BACK:
                targetLimelight = findGoodLimelight();
                break;
        }
        return targetLimelight == null;
    }

    private double getXOutput() {
        return xOut;
    }
    private double getYOutput() {
        return yOut;
    }
    private double getRotOutput() {
        return rotOut;
    }

    private void setTarget() {
        switch(target) {
            case LEFT:
                if(!setTargetLimelight(target)) {
                    break;
                }
                xPID.setGoal(LeftLimelight.kCloseXSetpoint);
                yPID.setGoal(LeftLimelight.kLeftYSetpoint);
                rotPID.setGoal(LeftLimelight.kRotSetpoint);
                break;
            case RIGHT:
                if(!setTargetLimelight(target)) {
                    break;
                }
                xPID.setGoal(RightLimelight.kCloseXSetpoint);
                yPID.setGoal(RightLimelight.kRightYSetpoint);
                rotPID.setGoal(RightLimelight.kRotSetpoint);
                break;
            case CENTER:
                if(!setTargetLimelight(target)) {
                    break;
                }
                // If the good limelight is the left one
                if(targetLimelight.getName().equals(limelightBeta.kName)) {
                    xPID.setGoal(LeftLimelight.kCloseXSetpoint);
                    yPID.setGoal(LeftLimelight.kCenterYSetpoint);
                    rotPID.setGoal(LeftLimelight.kRotSetpoint);
                }
                else if(targetLimelight.getName().equals(limelightAlpha.kName)) {
                    xPID.setGoal(RightLimelight.kCloseXSetpoint);
                    yPID.setGoal(RightLimelight.kCenterYSetpoint);
                    rotPID.setGoal(RightLimelight.kRotSetpoint);
                }
                break;
            case BACK:
                if(!setTargetLimelight(target)) {
                    break;
                }
                // If the good limelight is the left one
                if(targetLimelight.getName().equals(limelightBeta.kName)) {
                    xPID.setGoal(LeftLimelight.kFarXSetpoint);
                    yPID.setGoal(LeftLimelight.kCenterYSetpoint);
                    rotPID.setGoal(LeftLimelight.kRotSetpoint);
                }
                else if(targetLimelight.getName().equals(limelightAlpha.kName)) {
                    xPID.setGoal(RightLimelight.kFarXSetpoint);
                    yPID.setGoal(RightLimelight.kCenterYSetpoint);
                    rotPID.setGoal(RightLimelight.kRotSetpoint);
                }
                break;
            }
        
        m_vision.reefDriveTarget = target.toString();
        if(xPID.getGoal() == null || yPID.getGoal() == null || rotPID.getGoal() == null) {
            valid = false;
        }
        else {
            valid = true;
        }
    }


    private void resetAll() {
        xPID.reset(targetLimelight.getCameraPoseTS3d().toPose2d().getX(), getSpeeds().vxMetersPerSecond);
        yPID.reset(targetLimelight.getCameraPoseTS3d().toPose2d().getY(), getSpeeds().vyMetersPerSecond);
        rotPID.reset(targetLimelight.getCameraPoseTS3d().toPose2d().getRotation().getDegrees(), getSpeeds().omegaRadiansPerSecond);

    }
    private void shutdownAll() {
        shutdownX(); shutdownY(); shutdownRot();
    }
    private void shutdownX() {
        xPID.setGoal(targetLimelight.getCameraPoseTS3d().toPose2d().getX());
    }
    private void shutdownY() {
        yPID.setGoal(targetLimelight.getCameraPoseTS3d().toPose2d().getY());
    }
    private void shutdownRot() {
        rotPID.setGoal(targetLimelight.getCameraPoseTS3d().toPose2d().getRotation().getDegrees());
    }

    private ChassisSpeeds getSpeeds() {
        return m_swerve.getRobotRelativeSpeeds();
    }
}