package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.SK25Elevator;

public class TopBranchCommand extends Command
{
    @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
    private final SK25Elevator elevator;
    double kElevatorUpSpeed;

    public TopBranchCommand(SK25Elevator elevator)  //TODO: Add end effector to all elevator commands
    {
        this.elevator = elevator;
        

        //require a present subsystem in the json subsystems file
        addRequirements(elevator);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() 
    {

    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() 
    {
        if (elevator.atTop())
        {
            
        }
        else
        {
            elevator.runLeftMotor(kElevatorUpSpeed);
            elevator.runRightMotor(kElevatorUpSpeed);
        }
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) 
    {
        //stop the motor
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished()
    {
        return false;
    }
}