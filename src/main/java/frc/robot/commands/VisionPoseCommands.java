package frc.robot.commands;


import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.vision.SK25Vision;

public class VisionPoseCommands {
    static SK25Vision m_vision;
    public static Command autonResetPoseToVision() {
        return m_vision.runOnce(m_vision::autonResetPoseToVision)
        .withName("AutonResetPoseToVision");
    }

    public static Command resetPoseToVision() {
        return m_vision.runOnce(m_vision::resetPoseToVision)
        .withName("ResetPoseToVision");
    }
}
