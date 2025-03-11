package frc.robot.bindings;

import static frc.robot.Konstants.VisionConstants.kLeftSideReefAlignOffset;
import static frc.robot.Konstants.VisionConstants.kRightSideReefAlignOffset;
// import static frc.robot.Ports.OperatorPorts.kAlignToLeftReefCommand;
import static frc.robot.Ports.OperatorPorts.kAlignToRightReefCommand;
import static frc.robot.Ports.OperatorPorts.kMoveToSourceCommand;
// import frc.robot.RobotContainer.m_vision;

import java.util.Optional;

import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.VisionAlignCommands;
import frc.robot.commands.DriveToPose.DriveToPoseTranslationCommand;
import frc.robot.subsystems.SKSwerve;
import frc.robot.subsystems.vision.SK25Vision;
import frc.robot.utils.Field;
import static frc.robot.Ports.DriverPorts.kTranslationXPort;
import static frc.robot.Ports.DriverPorts.kTranslationYPort;
import static frc.robot.Ports.DriverPorts.kVelocityOmegaPort;

public class SK25VisionBinder implements CommandBinder {
    Optional<SK25Vision> m_visionContainer;
    Optional<SKSwerve> m_swerveContainer;

    Trigger moveToSourceButton;
    Trigger alignToLeftReefButton;
    Trigger alignToRightReefButton;

    public SK25VisionBinder(Optional<SK25Vision> m_visionContainer, Optional<SKSwerve> m_swerveContainer) {
        this.m_visionContainer = m_visionContainer;
        this.m_swerveContainer = m_swerveContainer;

        this.moveToSourceButton = kMoveToSourceCommand.button;
        // this.alignToLeftReefButton = kAlignToLeftReefCommand.button;
        this.alignToRightReefButton = kAlignToRightReefCommand.button;
    }

    public void bindButtons() {
        if(m_visionContainer.isPresent() && m_swerveContainer.isPresent()) {
            // The specific swerve instance is needed in order to control the robot
            // while the vision commands are all static since vision doesn't need one specific
            // instance to be controlled. Vision should be able to run multiple commands
            // either in sequence or parallel with itself.
            SKSwerve m_swerve = m_swerveContainer.get();



            moveToSourceButton.whileTrue(new DriveToPoseTranslationCommand(
                    SK25Vision.AlignTranslationWithPose.getConfig(),
                    Field.CoralStation.leftCenterFace,
                    () -> (kVelocityOmegaPort.getFilteredAxis()),
                    m_swerve));

            alignToLeftReefButton.whileTrue(VisionAlignCommands.visionAlignToLeftReef());

            alignToRightReefButton.whileTrue(VisionAlignCommands.visionAlignToRightReef());
        }
    }

    // public static Command autonResetPoseToVision() {
    //     return m_vision.runOnce(m_vision::autonResetPoseToVision);
    // }

    // public static Command visionAlignToLeftReef() {
    //     return new AlignToVisionTargetCommand(
    //             SK25Vision.AlignToReefTag.getConfig(), // AlignToTarget type config
    //             () -> (kTranslationYPort.getFilteredAxis()),
    //             kLeftSideReefAlignOffset);
    // }

    // public static Command visionAlignToRightReef() {
    //     return new AlignToVisionTargetCommand(
    //             SK25Vision.AlignToReefTag.getConfig(),
    //             () -> (kTranslationYPort.getFilteredAxis()),
    //             kRightSideReefAlignOffset);
    // }
}
