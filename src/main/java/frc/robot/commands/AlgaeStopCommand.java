package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.AlgaePickup;
public class AlgaeStopCommand extends Command{
    @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
    private final AlgaePickup Subsystem;
    public AlgaeStopCommand(AlgaePickup Subsystem){
        this.Subsystem = Subsystem;
    }
    public void initialize(){
        Subsystem.stopro();

    }
    public boolean isFinished(){
        return true;
    }
}