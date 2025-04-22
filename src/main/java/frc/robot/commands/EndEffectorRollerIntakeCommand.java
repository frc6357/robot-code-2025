package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Konstants.ElevatorConstants.CoralSubsystemConstants.ElevatorSetpoints;
import frc.robot.subsystems.CoralSubsystem;
import frc.robot.subsystems.SK25EndEffector;

import static frc.robot.Konstants.EndEffectorConstants.kRollerSlowSpeed;
import static frc.robot.Konstants.EndEffectorConstants.kRollerSpeed;


public class EndEffectorRollerIntakeCommand extends Command {

    private final SK25EndEffector Subsystem;
    private final CoralSubsystem elevator;
    public EndEffectorRollerIntakeCommand(SK25EndEffector Subsystem, CoralSubsystem elevator)
    {
        this.Subsystem = Subsystem;
        this.elevator = elevator;
    }

    public void initialize()
    {
        // Subsystem.runRoller(-kRollerSpeed);

        if ((elevator.elevatorCurrentTarget >= (ElevatorSetpoints.kLevel3 - 0.4)) && (elevator.elevatorCurrentTarget <= ElevatorSetpoints.kLevel3 + 0.4))
            Subsystem.runRoller(-kRollerSlowSpeed);
        else if ((elevator.elevatorCurrentTarget >= (ElevatorSetpoints.kLevel2 - 0.4)) && (elevator.elevatorCurrentTarget <= ElevatorSetpoints.kLevel2 + 0.4))
            Subsystem.runRoller(-kRollerSlowSpeed);
        else if ((elevator.elevatorCurrentTarget >= (ElevatorSetpoints.kLevel1 - 0.4)) && (elevator.elevatorCurrentTarget <= ElevatorSetpoints.kLevel1 + 0.4))
            Subsystem.runRoller(-kRollerSlowSpeed);
        // else if ((elevator.elevatorCurrentTarget >= (ElevatorSetpoints.kNet - 0.4)) && (elevator.elevatorCurrentTarget <= ElevatorSetpoints.kNet + 0.4)) //TODO: remove and replace in auto
        //     Subsystem.runRoller(-kRollerSuperSpeed);
        else
            Subsystem.runRoller(-kRollerSpeed);
    }

    // public void end(boolean interrupted)
    // {
    //     Subsystem.stopRoller();
    // }

    public boolean isFinished()
    {
        return true;
    }
    
}
