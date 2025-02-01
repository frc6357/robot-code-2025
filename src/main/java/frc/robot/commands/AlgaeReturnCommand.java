package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.AlgaePickup;
public class AlgaeReturnCommand extends Command {
    @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
    private final AlgaePickup Subsystem;
    public AlgaeReturnCommand(AlgaePickup Subsystem){
        this.Subsystem = Subsystem;
    }
    public void initialize(){
        Subsystem.upper();

    }
    public boolean isFinished(){
        return true;
    }
}

