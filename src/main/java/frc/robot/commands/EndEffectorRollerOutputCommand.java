package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.EndEffectorV2;

import static frc.robot.Konstants.EndEffectorConstants.kRollerSpeed;


public class EndEffectorRollerOutputCommand extends Command {

    private final EndEffectorV2 Subsystem;
    public EndEffectorRollerOutputCommand(EndEffectorV2 Subsystem)
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