// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.Konstants.ElevatorConstants.ElevatorPosition;
//import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.*;
import frc.robot.subsystems.SK25Elevator;

public class ElevatorButtonCommand extends Command
{
    @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})

    private final SK25Elevator      Elevator;
    private final ElevatorPosition  position;

    /**
     * This command allows the operator to set the angle of the arm to a specified
     * position.
     * 
     * @param position
     *            The position to set the arm to
     * @param Arm
     *            The Arm subsystem the command operates on.
     */
    public ElevatorButtonCommand(ElevatorPosition position, SK25Elevator Elevator)
    {
        this.position = position;
        this.Elevator = Elevator;
        // Use addRequirements() here to declare subsystem dependencies.
        addRequirements(Elevator);
    }

    @Override
    public void initialize()
    {
        Elevator.setTargetHeight(position);
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished()
    {
        return false;
        /*
        if(DriverStation.isAutonomousEnabled())
        {
            return true;
            //return Elevator.isAtTargetAngle();
        }
        else
        {
            return true;
        }
        */
        
    }
}