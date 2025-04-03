package frc.robot.bindings;

// import frc.robot.RobotContainer.m_vision;

import java.util.Optional;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.DriveToReef.DriveToReefPoseCommand;
import frc.robot.commands.AlignToReefTag;
import frc.robot.commands.commandGroups.AlignToReefComboTeleop;
import static frc.robot.commands.AlignToReefTag.Target;
import frc.robot.subsystems.SKSwerve;
import frc.robot.subsystems.vision.SK25Vision;
import static frc.robot.subsystems.vision.SK25Vision.DriveToPose;
import static frc.robot.subsystems.vision.SK25Vision.RotateToPose;


import static frc.robot.Ports.DriverPorts.kAlignToReef;
import static frc.robot.Ports.DriverPorts.kLeftReef;
import static frc.robot.Ports.DriverPorts.kResetPoseToVision;
import static frc.robot.Ports.DriverPorts.kRightReef;
import static frc.robot.Ports.DriverPorts.kVisionOff;
import static frc.robot.Ports.DriverPorts.kVisionOn;
import static frc.robot.Ports.DriverPorts.kForceResetPoseToVision;


public class SK25VisionBinder implements CommandBinder {
    Optional<SK25Vision> m_visionContainer;
    Optional<SKSwerve> m_swerveContainer;

    Trigger alignToReef;
    Trigger leftReef;
    Trigger rightReef;
    Trigger forceResetPoseToVision;
    Trigger resetPoseToVision;
    Trigger visionOff;
    Trigger visionOn;
    Trigger visionEnabled;

    public SK25VisionBinder(Optional<SK25Vision> m_visionContainer, Optional<SKSwerve> m_swerveContainer) {
        this.m_visionContainer = m_visionContainer;
        this.m_swerveContainer = m_swerveContainer;

        this.visionOff = kVisionOff.button;
        this.visionOn = kVisionOn.button;
        this.alignToReef = kAlignToReef.button;
        this.leftReef = kLeftReef.button;
        this.rightReef = kRightReef.button;
        this.forceResetPoseToVision = kForceResetPoseToVision.button;
        this.resetPoseToVision = kResetPoseToVision.button;
    }

    public void bindButtons() {
        if(m_visionContainer.isPresent() && m_swerveContainer.isPresent()) {
            // The specific swerve instance is needed in order to control the robot
            // while the vision commands are all static since vision doesn't need one specific
            // instance to be controlled. Vision should be able to run multiple commands
            // either in sequence or parallel with itself.
            SKSwerve m_swerve = m_swerveContainer.get();
            SK25Vision m_vision = m_visionContainer.get();

            visionEnabled = new Trigger(() -> m_vision.enabled);

            forceResetPoseToVision.and(visionEnabled).onTrue(new InstantCommand(() -> m_vision.forcePoseToVision()));
            resetPoseToVision.and(visionEnabled).onTrue(new InstantCommand(() -> m_vision.resetPoseToVision()));

            visionOff.onTrue(new InstantCommand(() -> m_vision.killVision()));
            visionOn.onTrue(new InstantCommand(() -> m_vision.enableVision()));

            // If just alignToReef held and not the other buttons
            alignToReef.and(visionEnabled).onTrue(
                new AlignToReefComboTeleop(
                    Target.CENTER, 
                    DriveToPose.getConfig(),
                    RotateToPose.getConfig(),
                    m_vision,
                    m_swerve)
            );
            leftReef.and(visionEnabled).onTrue(
                new AlignToReefComboTeleop(
                    Target.LEFT, 
                    DriveToPose.getConfig(), 
                    RotateToPose.getConfig(), 
                    m_vision, 
                    m_swerve)
            );
            rightReef.and(visionEnabled).onTrue(
                new AlignToReefComboTeleop(
                    Target.RIGHT, 
                    DriveToPose.getConfig(), 
                    RotateToPose.getConfig(), 
                    m_vision, 
                    m_swerve)
            );
        }
    }
}
