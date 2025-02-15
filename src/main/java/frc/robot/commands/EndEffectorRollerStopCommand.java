package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.EndEffectorV2;

public class EndEffectorRollerStopCommand extends Command{
    private final EndEffectorV2 Subsystem;
    public EndEffectorRollerStopCommand(EndEffectorV2 Subsystem)
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