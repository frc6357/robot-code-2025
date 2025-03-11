package frc.robot.commands;

import java.util.function.Supplier;

import frc.robot.subsystems.vision.SK25Vision.CommandConfig;

/**
 * Provides a general framework for vision commands, notably pose targetting
 * and other object targetting
 */
public abstract class VisionCommand {
    public abstract void initialize();
    public abstract void end();
    public abstract boolean atTarget();
    public abstract double getOutput();
}
