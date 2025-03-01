package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.SK25EndEffector;

public class EndEffectorStop extends Command{
    private final SK25EndEffector Subsystem;
    public EndEffectorStop(SK25EndEffector Subsystem)
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
