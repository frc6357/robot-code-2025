package frc.robot.commands;

import static frc.robot.Ports.OperatorPorts.kAlgaeGrab;
import static frc.robot.Ports.OperatorPorts.kAlgaeRelease;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.subsystems.AlgaePickup;

public class AlgaeCommand extends Command{
    Trigger Release;
    Trigger Grab;
    @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
    private final AlgaePickup Subsystem;
    public AlgaeCommand(AlgaePickup Subsystem){
        this.Release = kAlgaeRelease.button;
        this.Grab =  kAlgaeGrab.button;
        this.Subsystem = Subsystem;

        //require a present subsystem in the json subsystems file
        addRequirements(Subsystem);
    }
    @Override
    public void initialize() 
    {
        
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() 
    {
        Subsystem.lower();
        Subsystem.grab();
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) 
    {
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished()
    {
        return true;
    }

}
