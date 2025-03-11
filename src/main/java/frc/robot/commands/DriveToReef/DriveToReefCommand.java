package frc.robot.commands.DriveToReef;

import static edu.wpi.first.units.Units.Micro;

import java.util.ArrayList;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Ports.OperatorPorts;
import frc.robot.commands.DriveCommand;
import frc.robot.subsystems.SKSwerve;
import frc.robot.subsystems.vision.SK25Vision;
import frc.robot.subsystems.vision.SK25Vision.CommandConfig;
import frc.robot.subsystems.vision.SK25Vision.MultiLimelightCommandConfig;
import frc.robot.utils.vision.Limelight;
import frc.robot.utils.vision.LimelightHelpers.RawFiducial;
import static frc.robot.Ports.DriverPorts;

public class DriveToReefCommand extends Command{
    MultiLimelightCommandConfig driveConfig;
    MultiLimelightCommandConfig rotateConfig;

    Limelight[] limelights;

    RotateToReef rotateController;
    TranslateToReef driveController;

    Pose2d targetPose;

    SK25Vision m_vision;
    SKSwerve m_swerve;

    DriveCommand driveCommand;

    boolean valid = true;

    public DriveToReefCommand(MultiLimelightCommandConfig driveConfig, MultiLimelightCommandConfig rotateConfig, Pose2d targetPose, SK25Vision m_vision, SKSwerve m_swerve) {
        this.targetPose = targetPose;
        this.driveConfig = driveConfig;
        this.rotateConfig = rotateConfig;
        this.m_swerve = m_swerve;
        this.m_vision = m_vision;

        this.rotateController = new RotateToReef(rotateConfig, targetPose, m_swerve);
        this.driveController = new TranslateToReef(driveConfig, targetPose, m_swerve);

        // Since we will be driving and rotating at the same time, the drive type will need to be field-centric
        this.driveCommand = new DriveCommand(
            () -> (driveController.getXOutput()),
            () -> (driveController.getYOutput()), 
            () -> (rotateController.getOutput()), 
            () -> (true));
        
        // Drive config and rotate config both use the same limelights, so only need to call one config's array here
        this.limelights = driveConfig.limelights;

        addRequirements(m_swerve);
    }

    @Override
    public void initialize() {
        ArrayList<RawFiducial> reefTags = new ArrayList<RawFiducial>();
        RawFiducial closestTag = null;

        for(Limelight ll : limelights) {
            RawFiducial closestReefTag = m_vision.getClosestTargetFiducial(ll, SK25Vision.FIELD_ELEMENT.REEF);
            // getClosestTargetFiducial() will return null if no targets seen
            if(closestReefTag != null) {
                reefTags.add(closestReefTag);
            }
        }

        // Assuming that there is a valid reef tag in sight:
        if(!reefTags.isEmpty()) {
            closestTag = reefTags.get(0);
            double closestTagDistance = closestTag.distToRobot;

            if(reefTags.size() > 1) { // No need to for loop if only one tag seen
                for(RawFiducial reefTag : reefTags) {
                    if(reefTag.distToRobot < closestTagDistance) {
                        closestTag = reefTag;
                        closestTagDistance = closestTag.distToRobot;
                    }
                }
            }
        }
        else {
            valid = false;
            DriverStation.reportWarning("DriveToReefCommand failed. No reef tags seen", false);
        }

        if(valid) {
            driveController.initialize();
            rotateController.initialize();
        }
    }

    @Override
    public void execute() {
        if(valid) {
            // We don't want the controller rumbling during auto
            if(DriverStation.isTeleopEnabled()) {
                DriverPorts.kDriver.setRumble(RumbleType.kBothRumble, 0.5);
            }
            driveCommand.run();
        }
        else {
            DriverPorts.kDriver.setRumble(RumbleType.kBothRumble, 0.0);
        }
    }

    @Override
    public boolean isFinished() {
        if(!valid) {
            return true;
        }
        return (driveController.isFinished() && rotateController.isFinished());
    }

    @Override
    public void end(boolean isInterrupted) {
        DriverPorts.kDriver.setRumble(RumbleType.kBothRumble, 0.0);
        driveController.end();
        rotateController.end();
        if(!isInterrupted && DriverStation.isTeleopEnabled()) {
            // If in teleop, provide a short rumble to the operator to signal the command's completion
            OperatorPorts.kOperator.setRumble(RumbleType.kBothRumble, 0.5);
            Timer.delay(0.15);
            OperatorPorts.kOperator.setRumble(RumbleType.kBothRumble, 0.0);
        }
    }

}
