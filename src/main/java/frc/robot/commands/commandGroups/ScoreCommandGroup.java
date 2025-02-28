package frc.robot.commands.commandGroups;

//import static frc.robot.Constants.LauncherConstants.kAmpDefaultLeftSpeed;
//import static frc.robot.Constants.LauncherConstants.kAmpDefaultRightSpeed;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;

import frc.robot.Konstants.ElevatorConstants.ElevatorPosition;
import frc.robot.Konstants.EndEffectorConstants.EndEffectorPosition;

//import frc.robot.commands.LaunchCommandAuto;
import frc.robot.commands.ElevatorButtonCommand;
import frc.robot.commands.EndEffectorButtonCommand;
import frc.robot.subsystems.SK25Elevator;
import frc.robot.subsystems.EndEffectorV2;
import edu.wpi.first.wpilibj2.command.InstantCommand;

public class ScoreCommandGroup extends ParallelCommandGroup
{
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})

    private final ElevatorPosition  elevatorPos;
    private final EndEffectorPosition  endEffectorPos;

  /**
   * Command to score any level of the reef, using both the elevator & end effector.
   * @param elevator    Elevator subystem to use
   * @param endEffector Endeffector subsystem to use
   */
  public ScoreCommandGroup(ElevatorPosition elevatorPos, SK25Elevator elevator, EndEffectorPosition endEffectorPos, EndEffectorV2 endEffector) 
  {
    this.endEffectorPos = endEffectorPos;
    this.elevatorPos = elevatorPos;
    addCommands(
        new ElevatorButtonCommand(elevatorPos, elevator),
        new EndEffectorButtonCommand(endEffectorPos, endEffector)
        );
  }

}
