// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.Konstants.ElevatorConstants.ElevatorPosition;
import frc.robot.subsystems.SK25Elevator;

public class ElevatorButtonCommand extends Command
{
    @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})

    private final SK25Elevator      elevator;
    private final ElevatorPosition  pos;

    /**
     * This command allows the operator to set the angle of the arm to a specified
     * position.
     * 
     * @param position
     *            The position to set the arm to
     * @param Elevator
     *            The Elevator subsystem the command operates on.
     */
    public ElevatorButtonCommand(ElevatorPosition pos, SK25Elevator elevator)
    {
        this.pos = pos;
        this.elevator = elevator;
        // Use addRequirements() here to declare subsystem dependencies.
        addRequirements(elevator);
    }

    @Override
    public void initialize()
    {
        elevator.setTargetHeight(pos);
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished()
    {
        return elevator.isAtTargetPosition();
    }
    public void end(boolean interrupted) {
        System.out.println("Finished " + elevator.encoder.getPosition());
    }
}