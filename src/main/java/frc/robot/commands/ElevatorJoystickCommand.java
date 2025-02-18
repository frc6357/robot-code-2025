// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import static frc.robot.Konstants.ElevatorConstants.*;

import java.util.function.Supplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.SK25Elevator;

public class ElevatorJoystickCommand extends Command {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})

    private final SK25Elevator elevator;
    private final Supplier<Double>  controller;
    private final Supplier<Boolean> override;

    /**
     * Sets the angle of the arm based upon input from a joystick, adding or subtracting
     * to the current set point. Default movement will receive joystick input with
     * downward movement on joystick turning motor clockwise and upward movement on
     * joystick turning motor counter clockwise.
     * 
     * @param setpointChange
     *            The method to get the setpoint change in degrees per second
     * @param clampOverride
     *            The method to determine if the angle limits should be overridden
     * @param elevator
     *            Subsystem used for this command
     */

    public ElevatorJoystickCommand(Supplier<Double> setpointChange, Supplier<Boolean> clampOverride, SK25Elevator elevator)
    {
        this.controller = setpointChange;
        this.override = clampOverride;
        this.elevator = elevator;
        // Use addRequirements() here to declare subsystem dependencies.
        addRequirements(elevator);
    }

    @Override
    public void initialize(){}

    @Override
    public void execute()
    {
        double positionChange = controller.get() / 50; // Units per 20ms (20ms x 50 = 1s)

         // Sets the new height to the current position plus or minus the constant change
        // double rightSetpoint = elevator.getRightTargetPosition() + positionChange;
        double setpoint = elevator.getTargetPosition() + positionChange;

        if(!override.get())
        {
            /* 
            Functions like an if statement could, makes the setpoint value stay 
            within the minimum and maximum values that are set in Konstants.
            */

            // rightSetpoint = MathUtil.clamp(rightSetpoint, kMinHeight, kMaxHeight);
            setpoint = MathUtil.clamp(setpoint, kMinHeight, kMaxHeight);
        }

        // These methods bring the motors up to the setpoint created above.
        elevator.setTargetHeight(setpoint);
        //elevator.setLeftTargetHeight(leftSetpoint);
    }

    @Override
    public void end(boolean interrupted){}

    // Returns true when the command should end.
    @Override
    public boolean isFinished()
    {
        return false;
    }
}
