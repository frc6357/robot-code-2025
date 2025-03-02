package frc.robot.commands;

import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.vision.SK25Vision.CommandConfig;
import frc.robot.utils.vision.Limelight;
import frc.robot.commands.DriveCommand;

public class AlignToVisionTarget extends Command {
    private static CommandConfig config;
    private static Limelight limelight;
    DriveCommand driveCommand;
    Supplier<Double> fwdPositiveSupplier; // This is a robot centric foward/backwards
    private static double out = 0;
    private double heading = Integer.MIN_VALUE;
    private double horizontalSetpoint; // Goal tx of vision target

    // Because driving with vision is robot centric, aligning involves specifically only moving on the robot's x-plane to align with a target

    public AlignToVisionTarget(CommandConfig config, Supplier<Double> fwdPositiveSupplier, double horizontalSetpoint) {
        
    }
}
