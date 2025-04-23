package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Konstants.ElevatorConstants.CoralSubsystemConstants.ElevatorSetpoints;
import frc.robot.subsystems.CoralSubsystem;
import frc.robot.subsystems.SK25EndEffector;

import static frc.robot.Konstants.EndEffectorConstants.kRollerSlowSpeed;
import static frc.robot.Konstants.EndEffectorConstants.kRollerSpeed;


public class EndEffectorRollerOutputCommand extends Command {

    private final SK25EndEffector Subsystem;
    private final CoralSubsystem elevator;
    public EndEffectorRollerOutputCommand(SK25EndEffector Subsystem, CoralSubsystem elevator)
    {
        this.Subsystem = Subsystem;
        this.elevator = elevator;
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