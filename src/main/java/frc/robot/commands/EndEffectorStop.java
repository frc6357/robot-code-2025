package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.EndEffectorV2;

public class EndEffectorStop extends Command{
    private final EndEffectorV2 Subsystem;
    public EndEffectorStop(EndEffectorV2 Subsystem)
    {
        this.Subsystem = Subsystem;
    }

    public void initialize()
    {
        Subsystem.stopRoller();
    }
    public boolean isFinished()
    {
        return true;
    }
}
