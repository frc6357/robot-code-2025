package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.EndEffectorV1;

public class EndEffectorStopCommand extends Command{
    private final EndEffectorV1 Subsystem;
    public EndEffectorStopCommand(EndEffectorV1 Subsystem)
    {
        this.Subsystem = Subsystem;
    }

    public void initialize()
    {
        Subsystem.stopArm();
    }
    public boolean isFinished()
    {
        return true;
    }
    
}
