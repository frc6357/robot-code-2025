// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import static frc.robot.Konstants.ElevatorConstants.kMaxAngle;
import static frc.robot.Konstants.ElevatorConstants.kMinAngle;

import java.util.function.Supplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.SK25Elevator;
/** An example command that uses an example subsystem. */
public class ElevatorCommand extends Command {
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
     * @param climb
     *            Subsystem used for this command
     */
    public ElevatorCommand(Supplier<Double> setpointChange, Supplier<Boolean> clampOverride, SK25Elevator elevator)
    {
        this.controller = setpointChange;
        this.override = clampOverride;
        this.elevator = elevator;
        // Use addRequirements() here to declare subsystem dependencies.
        addRequirements(elevator);
    }

    @Override
    public void initialize()
    {}

    @Override
    public void execute()
    {
        double angleChange = controller.get() / 50; // Units per 20ms from 0.0 to 1.0

         // Sets the new angle to the current angle plus or minus the constant change
        double rightSetpoint = elevator.getRightTargetPosition() + angleChange;
        double leftSetpoint= elevator.getLeftTargetPosition() + angleChange;

        if(!override.get())
        {
            rightSetpoint = MathUtil.clamp(rightSetpoint, kMinAngle, kMaxAngle);
            leftSetpoint = MathUtil.clamp(leftSetpoint, kMinAngle, kMaxAngle);
        }

        elevator.setRightMotor(rightSetpoint);
        elevator.setLeftMotor(leftSetpoint);
    }

    @Override
    public void end(boolean interrupted)
    {

    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished()
    {
        return false;
    }
}
