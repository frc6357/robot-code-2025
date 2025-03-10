package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.SK25EndEffector;

import static frc.robot.Konstants.EndEffectorConstants.kRollerIntakeSpeed;


public class EndEffectorRollerIntakeCommand extends Command {

    private final SK25EndEffector Subsystem;
    public EndEffectorRollerIntakeCommand(SK25EndEffector Subsystem)
    {
        this.Subsystem = Subsystem;
    }

    public void initialize()
    {
        Subsystem.runRoller(kRollerIntakeSpeed);
    }


    public boolean isFinished()
    {
        return true;
    }
    
}
