package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.AlgaePickup;

public class AlgaeCommand extends Command{
    @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
    private final AlgaePickup Subsystem;
    private int _phase = 1;
    public AlgaeCommand(AlgaePickup Subsystem){
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
        if (_phase==1){
            Subsystem.lower();
            Subsystem.grab();
        }
        if (_phase==2){
            Subsystem.release();
        }
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) 
    {
        if (_phase==2){
            Subsystem.upper();
            _phase=0;
        }
        _phase++;
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished()
    {
        return false;
    }

}
