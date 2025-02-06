// package frc.robot.bindings;

// import java.util.Optional;

// import frc.robot.commands.OldSwerveCommand;
// import frc.robot.subsystems.OldSwerve;
// import static frc.robot.Ports.DriverPorts.*;

// public class OldSwerveBinder implements CommandBinder{
    
//     //create a subsystem toggleable by the json subsystems file
//     Optional <OldSwerve> m_swerve;

//     //create the ExampleButton trigger object 


//     //double leftX = kTranslationXPort.getFilteredAxis();
//     //double leftY = kTranslationYPort.getFilteredAxis();
//     //double rightX = kVelocityOmegaPort.getFilteredAxis();

//     public OldSwerveBinder(Optional<OldSwerve> swerve)
//     {
//         this.m_swerve = swerve;

//     }
     
//     public void bindButtons()
//     {
//         //if the subsytem is present in the json subsystems file
//         if (m_swerve.isPresent())
//         {
//             OldSwerve Swerve = m_swerve.get();

//             OldSwerveCommand swerveCommand = new OldSwerveCommand(
//             Swerve, 
//             () -> kTranslationXPort.getFilteredAxis(), 
//             () -> kTranslationYPort.getFilteredAxis(), 
//             () -> kVelocityOmegaPort.getFilteredAxis()
//             );

//             //make the swerveCommand run when no other command which utilizes the SK25Swerve subsytem runs.
//             //Swerve.setDefaultCommand(swerveCommand);


//         }
//     }

// }
