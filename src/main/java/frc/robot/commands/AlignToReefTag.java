package frc.robot.commands;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.SKSwerve;
import frc.robot.subsystems.vision.SK25Vision;
import frc.robot.subsystems.vision.SK25Vision.MultiLimelightCommandConfig;
import frc.robot.utils.vision.Limelight;
import frc.robot.utils.vision.LimelightHelpers;
import frc.robot.utils.vision.LimelightHelpers.RawFiducial;
import static frc.robot.Konstants.VisionConstants.AlignmentConstants.*;
import static frc.robot.Ports.DriverPorts.kDriver;

import java.util.Optional;

import static frc.robot.Konstants.VisionConstants.limelightAlpha;
import static frc.robot.Konstants.VisionConstants.limelightBeta;


public class AlignToReefTag extends Command {
    private ProfiledPIDController xPID, yPID, rotPID;
    private SKSwerve m_swerve;
    private MultiLimelightCommandConfig xyConfig;
    private MultiLimelightCommandConfig rotConfig;
    private Constraints xyConstraints, rotConstraints;
    private DriveCommand driveCommand;
    private SK25Vision m_vision;
    private Limelight[] limelights;
    private Limelight targetLimelight;
    private Target target;
    private boolean valid;

    private boolean outputtingX = true;
    private boolean outputtingY = true;
    private boolean outputtingRot = true;

    private double xOut;
    private double yOut;
    private double rotOut;

    private double tagID = -1;

    public static enum Target {
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

        xPID.setTolerance(xyConfig.tolerance);
        yPID.setTolerance(xyConfig.tolerance);
        rotPID.setTolerance(rotConfig.tolerance);
        
        this.driveCommand = new DriveCommand(
            () -> getXOutput(), 
            () -> getYOutput(), 
            () -> getRotOutput(), 
            () -> false);

        addRequirements(m_swerve);
    }

    @Override
    public void initialize() {
        setTarget();
        resetAll();
        outputtingX = true;
        outputtingY = true;
        outputtingRot = true;
    }

    @Override
    public void execute() {
        if(valid) {
            if(xPID.atSetpoint()) {
                outputtingX = false;
            }
            if(yPID.atSetpoint()) {
                outputtingY = false;
            }
            if(rotPID.atSetpoint()) {
                outputtingRot = false;
            }
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
        SmartDashboard.putNumber("Align/xPIDTarget", xPID.getGoal().position);
        SmartDashboard.putNumber("Align/yPIDTarget", yPID.getGoal().position);
        SmartDashboard.putNumber("Align/rotPIDTarget", rotPID.getGoal().position);
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
            boolean isAtTargetPose = xPID.atGoal() && yPID.atGoal() && rotPID.atGoal();
            boolean canNoLongerSeeTarget = !targetLimelight.targetInView();
            return isAtTargetPose || canNoLongerSeeTarget;
        }
        return !valid;
    }

    private void calculateSpeeds() {
        if(targetLimelight.getClosestTagID() == tagID) {
            double[] positions = targetLimelight.getRobotPoseTS();
    
            SmartDashboard.putNumber("Align/X", positions[2]);
            SmartDashboard.putNumber("Align/Y", positions[0]);
            SmartDashboard.putNumber("Align/RotDeg", positions[4]);
    
            if(outputtingX) {
                xOut = xyConfig.maxVelocity * xPID.calculate(positions[2]);
                if(Math.abs(xOut) < 0.2) {
                    xOut = Math.signum(xOut) * .2;
                }
            }
            else {
                xOut = 0;
            }
    
            if(outputtingY) {
                yOut = xyConfig.maxVelocity * -yPID.calculate(positions[0]);
                if(Math.abs(yOut) < 0.2) {
                    yOut = Math.signum(yOut) * .2;
                }
            }
            else {
                yOut = 0;
            }
    
            if(outputtingRot) {
                rotOut = rotConfig.maxVelocity * -rotPID.calculate(positions[4]);
                if(Math.abs(rotOut) < 0.05) {
                    rotOut = Math.signum(rotOut) * .05;
                }
            }
            else {
                rotOut = 0;
            }
        }
    }

    private Limelight findGoodLimelight() {
        for(Limelight ll : limelights) {
            if(m_vision.reefTargetClose(ll)) {
                return ll;
            }
        }
        return null;
    }

    private Optional<Limelight> getTargetLimelight(Target t) {
        switch(t) {
            case LEFT:
                if(!m_vision.reefTargetClose(limelights[1])) {
                    break;
                }
                return Optional.of(limelights[1]);
            case RIGHT:
                if(!m_vision.reefTargetClose(limelights[0])) {
                    break;
                }
                return Optional.of(limelights[0]);
            case CENTER:
                if(!m_vision.reefTargetClose(limelights[0])) {
                    break;
                }
                return Optional.of(limelights[0]);
            case BACK:
                return Optional.of(findGoodLimelight());
        }
        return Optional.empty();

    }

    private boolean setTargetLimelight(Target t) {
        getTargetLimelight(t).ifPresentOrElse(
            (ll) -> {
                targetLimelight = ll;
                tagID = targetLimelight.getClosestTagID();
            },
            () -> {
                targetLimelight = null;
                tagID = -1;
            }
        );
        
        return targetLimelight != null;
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
                xPID.setGoal(new State(kCloseXSetpoint, 0.0));
                yPID.setGoal(new State(kLeftYSetpoint, 0.0));
                rotPID.setGoal(new State(kRotSetpoint, 0.0));
                outputtingX = true; outputtingY = true; outputtingRot = true;
                break;
            case RIGHT:
                if(!setTargetLimelight(target)) {
                    break;
                }
                xPID.setGoal(kCloseXSetpoint);
                yPID.setGoal(kRightYSetpoint);
                rotPID.setGoal(kRotSetpoint);
                outputtingX = true; outputtingY = true; outputtingRot = true;
                break;
            case CENTER:
                if(!setTargetLimelight(target)) {
                    break;
                }
                // If the good limelight is the left one
                if(targetLimelight.getName().equals(limelightBeta.kName)) {
                    xPID.setGoal(kCloseXSetpoint);
                    yPID.setGoal(kCenterYSetpoint);
                    rotPID.setGoal(kRotSetpoint);
                    outputtingX = true; outputtingY = true; outputtingRot = true;
                }
                else if(targetLimelight.getName().equals(limelightAlpha.kName)) {
                    xPID.setGoal(kCloseXSetpoint);
                    yPID.setGoal(kCenterYSetpoint);
                    rotPID.setGoal(kRotSetpoint);
                    outputtingX = true; outputtingY = true; outputtingRot = true;
                }
                break;
            case BACK:
                if(!setTargetLimelight(target)) {
                    break;
                }
                // If the good limelight is the left one
                if(targetLimelight.getName().equals(limelightBeta.kName)) {
                    xPID.setGoal(kFarXSetpoint);
                    yPID.setGoal(kCenterYSetpoint);
                    rotPID.setGoal(kRotSetpoint);
                    outputtingX = true; outputtingY = true; outputtingRot = true;
                }
                else if(targetLimelight.getName().equals(limelightAlpha.kName)) {
                    xPID.setGoal(kFarXSetpoint);
                    yPID.setGoal(kCenterYSetpoint);
                    rotPID.setGoal(kRotSetpoint);
                    outputtingX = true; outputtingY = true; outputtingRot = true;
                }
                break;
            }
        
        m_vision.reefDriveTarget = target.toString();
        if(xPID.getGoal() == null || yPID.getGoal() == null || rotPID.getGoal() == null) {
            valid = false;
        }
        else {
            valid = targetLimelight != null;
        }
    }


    private void resetAll() {
        if(valid) {
            double[] positions = targetLimelight.getRobotPoseTS();
            xPID.reset(positions[2], getSpeeds().vxMetersPerSecond);
            yPID.reset(positions[0], getSpeeds().vyMetersPerSecond);
            rotPID.reset(positions[4], getSpeeds().omegaRadiansPerSecond);
        }

    }
    private void shutdownAll() {
        // shutdownX(); shutdownY(); shutdownRot();
    }
    private void shutdownX() {
        double[] positions = targetLimelight.getRobotPoseTS();
        xPID.setGoal(positions[2]);
    }
    private void shutdownY() {
        double[] positions = targetLimelight.getRobotPoseTS();
        yPID.setGoal(positions[0]);
    }
    private void shutdownRot() {
        double[] positions = targetLimelight.getRobotPoseTS();
        rotPID.setGoal(positions[4]);
    }

    private ChassisSpeeds getSpeeds() {
        return m_swerve.getRobotRelativeSpeeds();
    }
}