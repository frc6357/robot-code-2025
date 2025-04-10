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

    private double xOut;
    private double yOut;
    private double rotOut;

    private boolean xDone = false;
    private boolean yDone = false;
    private boolean rotDone = false;

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

        rotPID.enableContinuousInput(-180, 180);
        
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
    }

    @Override
    public void execute() {
        if(valid) {
            calculateSpeeds();

            m_vision.isDriving = true;

            if(DriverStation.isTeleopEnabled()) {
                if(xDone && yDone){
                    kDriver.setRumble(RumbleType.kBothRumble, 0.5);
                }
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
        
        if(valid) {
            shutdownAll();
        }
    }

    @Override
    public boolean isFinished() {
        if(valid) {
            boolean isAtTargetPose = xDone && yDone && rotDone;
            boolean canNoLongerSeeTarget = !targetLimelight.targetInView();
            boolean seesWrongTarget = targetLimelight.getClosestTagID() != tagID;
            return isAtTargetPose || canNoLongerSeeTarget || seesWrongTarget;
        }
        return !valid;
    }

    private void calculateSpeeds() {
        if(targetLimelight.getClosestTagID() == tagID) {
            double[] positions = targetLimelight.getRobotPoseTS();
    
            SmartDashboard.putNumber("Align/X", positions[2]);
            SmartDashboard.putNumber("Align/Y", positions[0]);
            SmartDashboard.putNumber("Align/RotDeg", positions[4]);
    
            // if(outputtingX) {
                 xOut = xyConfig.maxVelocity * xPID.calculate(positions[2]);
                if(Math.abs(positions[2] - xPID.getGoal().position) <= xyConfig.tolerance) {
                    xOut = 0;
                }
                else if(Math.abs(positions[2] - xPID.getGoal().position) < 0.05 && xOut < 0.1) {
                    xOut = 0.1 * Math.signum(xOut);
                }
                 xDone = (Math.abs(positions[2] - xPID.getGoal().position) <= xyConfig.tolerance);
                 //     if(Math.abs(xOut) < 0.2) {
                    //         xOut = Math.signum(xOut) * .2;
                    //     }
                    // }

            // else {
            //     xOut = 0;
            // }
            // if(outputtingY) {
                 yOut = xyConfig.maxVelocity * -yPID.calculate(positions[0]);
                 if(Math.abs(positions[0] - yPID.getGoal().position) <= xyConfig.tolerance) {
                    yOut = 0;
                }
                 if(Math.abs(positions[0] - yPID.getGoal().position) < 0.05 && yOut < 0.1) {
                       yOut = 0.1 * Math.signum(yOut);
                 }
                 yDone = (Math.abs(positions[0] - yPID.getGoal().position) <= xyConfig.tolerance);

            //     if(Math.abs(yOut) < 0.2) {
            //         yOut = Math.signum(yOut) * .2;
            //     }
            // }
            // else {
            //     yOut = 0;
            // }
    
            // if(outputtingRot) {

                rotOut = rotConfig.maxVelocity * -rotPID.calculate(positions[4]);
                if(Math.abs(positions[4] - rotPID.getGoal().position) < 1 && rotOut < 0.1) {
                   rotOut = 0.1 * Math.signum(rotOut);
                }
                rotDone = (Math.abs(positions[4] - rotPID.getGoal().position) <= rotConfig.tolerance);

            //     if(Math.abs(rotOut) < 0.05) {
            //         rotOut = Math.signum(rotOut) * .05;
            //     }
            // }
            // else {
            //     rotOut = 0;
            // }
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
                if(targetLimelight.getDistanceToTagFromCamera() > 3.5) {
                    break;
                }
                xPID.setGoal(new State(kCoralXSetpoint, 0.0));
                yPID.setGoal(new State(kLeftYSetpoint, 0.0));
                rotPID.setGoal(new State(kRotSetpoint, 0.0));
                break;
            case RIGHT:
                if(!setTargetLimelight(target)) {
                    break;
                }
                if(targetLimelight.getDistanceToTagFromCamera() > 3.5) {
                    break;
                }
                xPID.setGoal(kCoralXSetpoint);
                yPID.setGoal(kRightYSetpoint);
                rotPID.setGoal(kRotSetpoint);
                break;
            case CENTER:
                if(!setTargetLimelight(target)) {
                    break;
                }
                if(targetLimelight.getDistanceToTagFromCamera() > 3.5) {
                    break;
                }
                // If the good limelight is the left one
                if(targetLimelight.getName().equals(limelightBeta.kName)) {
                    xPID.setGoal(kAlgaeXSetpoint);
                    yPID.setGoal(kCenterYSetpoint);
                    rotPID.setGoal(kRotSetpoint);
                }
                else if(targetLimelight.getName().equals(limelightAlpha.kName)) {
                    xPID.setGoal(kAlgaeXSetpoint);
                    yPID.setGoal(kCenterYSetpoint);
                    rotPID.setGoal(kRotSetpoint);
                }
                break;
            case BACK:
                if(!setTargetLimelight(target)) {
                    break;
                }
                if(targetLimelight.getDistanceToTagFromCamera() > 3.5) {
                    break;
                }
                // If the good limelight is the left one
                if(targetLimelight.getName().equals(limelightBeta.kName)) {
                    xPID.setGoal(kFarXSetpoint);
                    yPID.setGoal(kCenterYSetpoint);
                    rotPID.setGoal(kRotSetpoint);
                }
                else if(targetLimelight.getName().equals(limelightAlpha.kName)) {
                    xPID.setGoal(kFarXSetpoint);
                    yPID.setGoal(kCenterYSetpoint);
                    rotPID.setGoal(kRotSetpoint);
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
        shutdownX(); shutdownY(); shutdownRot();
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