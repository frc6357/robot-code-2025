package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.SK25EndEffector;

import static frc.robot.Konstants.EndEffectorConstants.kRollerSpeed;


public class EndEffectorRollerOutputCommand extends Command {

    private final SK25EndEffector Subsystem;
    public EndEffectorRollerOutputCommand(SK25EndEffector Subsystem)
    {
        this.Subsystem = Subsystem;
    }

    public void initialize()
    {
        Subsystem.runRoller(kRollerSpeed);
    }


    public boolean isFinished()
    {
        return true;
    }
    
}