package frc.robot.commands.commandGroups;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.Konstants.EndEffectorConstants.EndEffectorPosition;
import frc.robot.commands.EndEffectorButtonCommand;
import frc.robot.subsystems.SK25Elevator;
import frc.robot.subsystems.SK25Elevator.ElevatorSetpoint;
import frc.robot.subsystems.SK25EndEffector;


public class LineupCombo extends ParallelCommandGroup
{

  /**
   * Command to score any level of the reef, using both the elevator & end effector.
   * @param elevator    Elevator subystem to use
   * @param endEffector Endeffector subsystem to use
   */
  public void ScoreCommandGroup(ElevatorSetpoint elevatorPos, SK25Elevator elevator, EndEffectorPosition endEffectorPos, SK25EndEffector endEffector) 
  {
      addCommands(
          new InstantCommand(() -> elevator.setSetpointCommand(elevatorPos)),
          new EndEffectorButtonCommand(endEffectorPos, endEffector)
      );
  }
}