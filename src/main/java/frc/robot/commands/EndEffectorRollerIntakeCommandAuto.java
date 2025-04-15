package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.SK25EndEffector;

import static frc.robot.Konstants.EndEffectorConstants.kRollerSpeed;


public class EndEffectorRollerIntakeCommandAuto extends Command {

    private final SK25EndEffector Subsystem;
    public EndEffectorRollerIntakeCommandAuto(SK25EndEffector Subsystem)
    {
        this.Subsystem = Subsystem;
    }

    public void initialize()
    {
        Subsystem.runRoller(kRollerSpeed);
    }

    public void end(boolean interrupted)
    {
        Subsystem.stopRoller();
    }

    public boolean isFinished()
    {
        return Subsystem.haveCoral();
    }
    
}
