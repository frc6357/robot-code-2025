package frc.robot.commands;

import static frc.robot.Konstants.VisionConstants.kLeftSideReefAlignOffset;
import static frc.robot.Konstants.VisionConstants.kRightSideReefAlignOffset;
import static frc.robot.Ports.DriverPorts.kTranslationYPort;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.vision.SK25Vision;

public class VisionAlignCommands {
    
    public static Command visionAlignToLeftReef() {
        return new AlignToVisionTargetCommand(
                SK25Vision.AlignToReefTag.getConfig(),
                () -> (kTranslationYPort.getFilteredAxis()),
                kLeftSideReefAlignOffset);
    }

    public static Command visionAlignToRightReef() {
        return new AlignToVisionTargetCommand(
                SK25Vision.AlignToReefTag.getConfig(),
                () -> (kTranslationYPort.getFilteredAxis()),
                kRightSideReefAlignOffset);
    }
}
