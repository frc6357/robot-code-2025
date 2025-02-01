package frc.robot.commands;


import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.AlgaePickup;
public class AlgaeReleaseCommand extends Command{
    @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
    private final AlgaePickup Subsystem;
    public AlgaeReleaseCommand(AlgaePickup Subsystem){
        this.Subsystem = Subsystem;

    }
    public void execute(){
        Subsystem.release();
    }
    public boolean isFinished(){
        return true;
    }
}