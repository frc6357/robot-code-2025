package frc.robot.commands.commandGroups;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.Konstants.EndEffectorConstants.EndEffectorPosition;
import frc.robot.commands.EndEffectorButtonCommand;
import frc.robot.commands.EndEffectorRollerIntakeCommand;
import frc.robot.commands.EndEffectorRollerStopCommand;
import frc.robot.subsystems.CoralSubsystem;
import frc.robot.subsystems.CoralSubsystem.Setpoint;
import frc.robot.subsystems.SK25EndEffector;


//Parallel Command Groups run all of their commands at the same time, and end when all of their 
//commands have ended. All commands in the group start imidiatley when the group is called.
public class StationCombo extends SequentialCommandGroup
{
  /**
   * Command to align with any known position, using both the elevator & end effector.
   * @param elevatorPos The setpoint of the elevator
   * @param elevator    Elevator subystem to use
   * @param endEffectorPos The angle of the endEffector
   * @param endEffector Endeffector subsystem to use
   */
  public StationCombo(Setpoint elevatorPos, CoralSubsystem elevator, EndEffectorPosition endEffectorPos, SK25EndEffector endEffector) 
  {
      addCommands(
        new ParallelDeadlineGroup
        (
          elevator.setSetpointCommand(elevatorPos),
          new EndEffectorButtonCommand(endEffectorPos, endEffector)
        ),
        new EndEffectorRollerIntakeCommand(endEffector),
        new WaitUntilCommand(endEffector::haveCoral),
        new EndEffectorRollerStopCommand(endEffector)
      );
  }
}
