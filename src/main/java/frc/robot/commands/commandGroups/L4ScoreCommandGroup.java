// package frc.robot.commands.commandGroups;
// import edu.wpi.first.wpilibj2.command.InstantCommand;
// import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
// import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
// import edu.wpi.first.wpilibj2.command.WaitCommand;
// // Subsystem constants (elevator & end effector)
// import frc.robot.Konstants.ElevatorConstants.ElevatorPosition;
// import frc.robot.Konstants.EndEffectorConstants.EndEffectorPosition;
// import static frc.robot.Konstants.EndEffectorConstants.kRollerSpeed;
// import static frc.robot.Konstants.EndEffectorConstants.kRollerStop;


// // Individual subsystem commands (elevator & end effector)
// import frc.robot.commands.ElevatorButtonCommand;
// import frc.robot.commands.EndEffectorButtonCommand;

// // Individual subsystems (elevator & end effector)
// import frc.robot.subsystems.SK25Elevator;
// import frc.robot.subsystems.SK25EndEffector;

// public class L4ScoreCommandGroup extends SequentialCommandGroup
// {
//   @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})

//     private final ElevatorPosition     elevatorPos;
//     private final EndEffectorPosition  endEffectorPos;

//   /**
//    * Command to score any level of the reef, using both the elevator & end effector.
//    * @param elevator    Elevator subystem to use
//    * @param endEffector Endeffector subsystem to use
//    */
//   public L4ScoreCommandGroup(ElevatorPosition elevatorPos, SK25Elevator elevator, EndEffectorPosition endEffectorPos, SK25EndEffector endEffector) 
//   {
//     this.endEffectorPos = endEffectorPos;
//     this.elevatorPos = elevatorPos;
    
//     addCommands(
//         new EndEffectorButtonCommand(EndEffectorPosition.kTopPositionAngle, endEffector),
//         new ElevatorButtonCommand(ElevatorPosition.kNetPosition, elevator),
//         new InstantCommand(() -> endEffector.runRoller(kRollerSpeed)),
//         new WaitCommand(.5),
//         new EndEffectorButtonCommand(EndEffectorPosition.kIntakePositionAngle, endEffector),
//         new InstantCommand(() -> endEffector.runRoller(kRollerStop))
//         );
//   }
// }
