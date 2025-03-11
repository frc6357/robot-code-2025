package frc.robot.commands.DriveToReef;

import java.util.ArrayList;
import java.util.List;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Ports.OperatorPorts;
import frc.robot.commands.DriveCommand;
import frc.robot.subsystems.SKSwerve;
import frc.robot.subsystems.vision.SK25Vision;
import frc.robot.subsystems.vision.SK25Vision.MultiLimelightCommandConfig;
import frc.robot.utils.Field;
import frc.robot.utils.vision.Limelight;
import frc.robot.utils.vision.LimelightHelpers.RawFiducial;
import static frc.robot.Ports.DriverPorts;
import frc.robot.Konstants.VisionConstants.PoseConstants;

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

    boolean previousTargetLeftStatus;
    boolean previousTargetRightStatus;

    Trigger targetLeftSide;
    Trigger targetRightSide;

    RawFiducial closestTag;


    public DriveToReefCommand(
                MultiLimelightCommandConfig driveConfig,
                MultiLimelightCommandConfig rotateConfig, 
                SK25Vision m_vision, 
                SKSwerve m_swerve,
                Trigger targetLeftSide,
                Trigger targetRightSide) {

        this.driveConfig = driveConfig;
        this.rotateConfig = rotateConfig;
        this.m_vision = m_vision;
        this.m_swerve = m_swerve;
        this.targetLeftSide = targetLeftSide;
        this.targetRightSide = targetRightSide;

        previousTargetLeftStatus = targetLeftSide.getAsBoolean();
        previousTargetRightStatus = targetRightSide.getAsBoolean();


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

    /* Switches the target of the reef pose while maintaining target reef face */
    private void setTargetPose() {
        List<String> targetPositions = PoseConstants.tagDestinationMap.get(closestTag.id);
            String targetPosition = targetPositions.get(2); // Default to middle face

            /* Effectively prioritizes left scoring */
            if(targetLeftSide.getAsBoolean()) {
                previousTargetLeftStatus = true;
                previousTargetRightStatus = false;
                targetPosition = targetPositions.get(0); // Left position is at index 0
            }
            else if(targetRightSide.getAsBoolean()) {
                previousTargetRightStatus = true;
                previousTargetLeftStatus = false;
                targetPosition = targetPositions.get(1); // Right position is at index 1
            }
            else {
                previousTargetLeftStatus = false;
                previousTargetRightStatus = false;
            }

            targetPose = PoseConstants.fieldPositions.get(targetPosition);

            targetPose = new Pose2d(
                        Field.flipXifRed(targetPose.getX()),
                        Field.flipYifRed(targetPose.getY()),
                        Field.flipAngleIfRed(targetPose.getRotation()));

            driveController.initialize(targetPose);
            rotateController.initialize(targetPose);
    }

    @Override
    public void initialize() {
        ArrayList<RawFiducial> reefTags = new ArrayList<RawFiducial>();
        closestTag = null;

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


        this.rotateController = new RotateToReef(rotateConfig, m_swerve);
        this.driveController = new TranslateToReef(driveConfig, m_swerve);

        if(valid) {
            setTargetPose();
        }
    }

    @Override
    public void execute() {
        if(valid) {
            m_vision.isDriving = true;
            // We don't want the controller rumbling during auto
            if(DriverStation.isTeleopEnabled()) {
                DriverPorts.kDriver.setRumble(RumbleType.kBothRumble, 0.5);
            }

            if(previousTargetLeftStatus != targetLeftSide.getAsBoolean()) {
                setTargetPose();
            }
            else if(previousTargetRightStatus != targetRightSide.getAsBoolean()) {
                setTargetPose();
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
        m_vision.isDriving = false;
        DriverPorts.kDriver.setRumble(RumbleType.kBothRumble, 0.0);
        driveController.end();
        rotateController.end();
        if(!isInterrupted && DriverStation.isTeleopEnabled()) {
            // If in teleop, provide a short rumble to the operator to signal the command's completion
            OperatorPorts.kOperator.setRumble(RumbleType.kBothRumble, 0.5);
            Timer.delay(0.15); // TODO: Make this not use Timer
            OperatorPorts.kOperator.setRumble(RumbleType.kBothRumble, 0.0);
        }
    }

}
