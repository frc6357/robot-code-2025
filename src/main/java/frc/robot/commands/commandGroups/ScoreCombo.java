package frc.robot.commands.commandGroups;
import static frc.robot.Konstants.EndEffectorConstants.kRollerSpeed;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Konstants.EndEffectorConstants.EndEffectorPosition;
import frc.robot.subsystems.SK25Elevator;
import frc.robot.subsystems.SK25Elevator.ElevatorSetpoint;
import frc.robot.subsystems.SK25EndEffector;


public class ScoreCombo extends ParallelCommandGroup
{
  /**
   * Command to score any level of the reef (and the net), using both the elevator & end effector.
   * @param elevatorPos The setpoint of the elevator
   * @param elevator    Elevator subystem to use
   * @param endEffectorPos The angle of the endEffector
   * @param endEffector Endeffector subsystem to use
   */
  public ScoreCombo(ElevatorSetpoint elevatorPos, SK25Elevator elevator, EndEffectorPosition endEffectorPos, SK25EndEffector endEffector) 
  {
      addCommands(
          new LineupCombo(elevatorPos, elevator, endEffectorPos, endEffector),
          //Parallel race groups run all of its commands periodicaly until every command has finished at 
          //least once. Since wait command is prolonged for some amount of time, the runRoller() method will
          //run for the amount of time specified in the wait command.
          new ParallelRaceGroup(new WaitCommand(0.5), new InstantCommand(() -> endEffector.runRoller(kRollerSpeed)))
      );
  }
}