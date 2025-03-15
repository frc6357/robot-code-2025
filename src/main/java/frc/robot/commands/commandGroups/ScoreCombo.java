package frc.robot.commands.commandGroups;

import static frc.robot.Konstants.AutoConstants.kExtakeAutoSpeed;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Konstants.EndEffectorConstants.EndEffectorPosition;
import frc.robot.subsystems.CoralSubsystem;
import frc.robot.subsystems.CoralSubsystem.Setpoint;
import frc.robot.subsystems.SK25EndEffector;

//Sequential Command Groups run all of their commands in the order they are listed, where the second
//dosn't start until the first ends.
public class ScoreCombo extends SequentialCommandGroup
{
  /**
   * Command to score any level of the reef (and the net), using both the elevator & end effector.
   * @param elevatorPos The setpoint of the elevator
   * @param elevator    Elevator subystem to use
   * @param endEffectorPos The angle of the endEffector
   * @param endEffector Endeffector subsystem to use
   */
  public ScoreCombo(Setpoint elevatorPos, CoralSubsystem elevator, EndEffectorPosition endEffectorPos, SK25EndEffector endEffector) 
  {
      addCommands(
          new LineupCombo(elevatorPos, elevator, endEffectorPos, endEffector),
          new WaitCommand(1.5), //seconds
          Commands.parallel(new WaitCommand(0.3), endEffector.runRollerCommand(kExtakeAutoSpeed)),
          new WaitCommand(0.5) //seconds
      );
  }
}