package frc.robot.bindings;

import static frc.robot.Ports.OperatorPorts.kReefAlignCommand;

import java.util.Optional;

import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.RobotContainer;
import frc.robot.commands.DriveToPoseTranslation;
import frc.robot.subsystems.SKSwerve;
import frc.robot.subsystems.vision.SK25Vision;
import frc.robot.utils.Field;

public class SK25VisionBinder implements CommandBinder {
    Optional<SK25Vision> m_visionContainer;
    Optional<SKSwerve> m_swerveContainer;

    Trigger reefAlignButton;

    public SK25VisionBinder(Optional<SK25Vision> m_visionContainer, Optional<SKSwerve> m_swerveContainer) {
        this.m_visionContainer = m_visionContainer;
        this.m_swerveContainer = m_swerveContainer;

        this.reefAlignButton = kReefAlignCommand.button;
    }

    public void bindButtons() {
        if(m_visionContainer.isPresent() && m_swerveContainer.isPresent()) {
            SK25Vision m_vision = m_visionContainer.get();
            SKSwerve m_swerve = m_swerveContainer.get();

            reefAlignButton.onTrue(new DriveToPoseTranslation(
                    SK25Vision.AlignWithPose.getConfig(),
                    Field.CoralStation.leftCenterFace,
                    () -> (m_swerve.getRobotRelativeSpeeds().omegaRadiansPerSecond),
                    m_swerve));
        }
    }
}
