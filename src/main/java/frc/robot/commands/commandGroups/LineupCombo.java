package frc.robot.commands.commandGroups;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.Konstants.EndEffectorConstants.EndEffectorPosition;
import frc.robot.commands.EndEffectorButtonCommand;
import frc.robot.subsystems.SK25Elevator;
import frc.robot.subsystems.SK25Elevator.ElevatorSetpoint;
import frc.robot.subsystems.SK25EndEffector;


//Parallel Command Groups run all of their commands at the same time, and end when all of their 
//commands have ended. All commands in the group start imidiatley when the group is called.
public class LineupCombo extends ParallelCommandGroup
{
  /**
   * Command to align with any known position, using both the elevator & end effector.
   * @param elevatorPos The setpoint of the elevator
   * @param elevator    Elevator subystem to use
   * @param endEffectorPos The angle of the endEffector
   * @param endEffector Endeffector subsystem to use
   */
  public LineupCombo(ElevatorSetpoint elevatorPos, SK25Elevator elevator, EndEffectorPosition endEffectorPos, SK25EndEffector endEffector) 
  {
      addCommands(
          new InstantCommand(() -> elevator.setSetpointCommand(elevatorPos)),
          new EndEffectorButtonCommand(endEffectorPos, endEffector)
      );
  }
}