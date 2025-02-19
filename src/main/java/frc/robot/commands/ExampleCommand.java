// package frc.robot.commands;

// import edu.wpi.first.wpilibj2.command.Command;
// import frc.robot.subsystems.ExampleSubsystem;

// public class ExampleCommand extends Command
// {
//     @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
//     private final ExampleSubsystem Subsystem;

//     public ExampleCommand(ExampleSubsystem Subsystem)
//     {
//         this.Subsystem = Subsystem;

//         //require a present subsystem in the json subsystems file
//         addRequirements(Subsystem);
//     }

//     // Called when the command is initially scheduled.
//     @Override
//     public void initialize() 
//     {
//         //run the motor
//        Subsystem.runMotor();
//     }

//     // Called every time the scheduler runs while the command is scheduled.
//     @Override
//     public void execute() 
//     {
//     }

//     // Called once the command ends or is interrupted.
//     @Override
//     public void end(boolean interrupted) 
//     {
//         //stop the motor
//         Subsystem.stopMotor();
//     }

//     // Returns true when the command should end.
//     @Override
//     public boolean isFinished()
//     {
//         return false;
//     }
// }

