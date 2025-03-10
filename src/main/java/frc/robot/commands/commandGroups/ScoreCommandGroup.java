// package frc.robot.commands.commandGroups;
// import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;

// // Subsystem constants (elevator & end effector)
// import frc.robot.Konstants.ElevatorConstants.ElevatorPosition;
// import frc.robot.Konstants.EndEffectorConstants.EndEffectorPosition;

// // Individual subsystem commands (elevator & end effector)
// import frc.robot.commands.ElevatorButtonCommand;
// import frc.robot.commands.EndEffectorButtonCommand;

// // Individual subsystems (elevator & end effector)
// import frc.robot.subsystems.SK25Elevator;
// import frc.robot.subsystems.SK25EndEffector;

// public class ScoreCommandGroup extends ParallelCommandGroup
// {
//   @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})

//     private final ElevatorPosition     elevatorPos;
//     private final EndEffectorPosition  endEffectorPos;

//   /**
//    * Command to score any level of the reef, using both the elevator & end effector.
//    * @param elevator    Elevator subystem to use
//    * @param endEffector Endeffector subsystem to use
//    */
//   public ScoreCommandGroup(ElevatorPosition elevatorPos, SK25Elevator elevator, EndEffectorPosition endEffectorPos, SK25EndEffector endEffector) 
//   {
//     this.endEffectorPos = endEffectorPos;
//     this.elevatorPos = elevatorPos;
    
//     addCommands(
//         new EndEffectorButtonCommand(endEffectorPos, endEffector),
//         new ElevatorButtonCommand(elevatorPos, elevator)
//         );
//   }
// }
