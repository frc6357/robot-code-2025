package frc.robot.commands.commandGroups;
import static frc.robot.Konstants.EndEffectorConstants.kRollerSpeed;

//import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Konstants.EndEffectorConstants.EndEffectorPosition;
// import frc.robot.commands.EndEffectorButtonCommand;
// import frc.robot.subsystems.CoralSubsystem;
// import frc.robot.subsystems.CoralSubsystem.Setpoint;
import frc.robot.subsystems.SK25EndEffector;


//Parallel Command Groups run all of their commands at the same time, and end when all of their 
//commands have ended. All commands in the group start imidiatley when the group is called.
public class IntakeAutoCommand extends SequentialCommandGroup
{
  /**
   * Command to align with any known position, using both the elevator & end effector.
   * @param endEffectorPos The angle of the endEffector
   * @param endEffector Endeffector subsystem to use
   */
  public IntakeAutoCommand(EndEffectorPosition endEffectorPos, SK25EndEffector endEffector) 
  {
      addCommands(
        new ParallelDeadlineGroup
        (
          endEffector.runRollerCommand(kRollerSpeed),
          new WaitCommand(3)
        ),
        endEffector.runRollerCommand(0)
      );
  }
}