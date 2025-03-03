package frc.robot.bindings;

import static frc.robot.Konstants.VisionConstants.kLeftSideReefOffset;
// import static frc.robot.Ports.OperatorPorts.kAlignToLeftReefCommand;
import static frc.robot.Ports.OperatorPorts.kAlignToRightReefCommand;
import static frc.robot.Ports.OperatorPorts.kMoveToSourceCommand;

import java.util.Optional;

import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.AlignToVisionTargetCommand;
import frc.robot.commands.DriveToPoseTranslationCommand;
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
            SK25Vision m_vision = m_visionContainer.get();
            SKSwerve m_swerve = m_swerveContainer.get();



            moveToSourceButton.whileTrue(new DriveToPoseTranslationCommand(
                    SK25Vision.AlignTranslationWithPose.getConfig(),
                    Field.CoralStation.leftCenterFace,
                    () -> (kVelocityOmegaPort.getFilteredAxis()),
                    m_swerve));

            alignToLeftReefButton.whileTrue(new AlignToVisionTargetCommand(
                    SK25Vision.AlignToReefTag.getConfig(), // AlignToTarget type config
                    () -> (kTranslationYPort.getFilteredAxis()),
                    kLeftSideReefOffset));
        }
    }
}
