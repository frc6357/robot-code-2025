package frc.robot.bindings;

import java.util.Optional;

import frc.robot.commands.SwerveCommand;
import frc.robot.subsystems.SK25SwerveFactory;
import static frc.robot.Ports.DriverPorts.*;

public class SK25SwerveBinder implements CommandBinder{
    
    //create a subsystem toggleable by the json subsystems file
    Optional <SK25SwerveFactory> m_factory;


    public SK25SwerveBinder(Optional<SK25SwerveFactory> optionalFactory)
    {
        this.m_factory = optionalFactory;
    }
     
    public void bindButtons()
    {
        //if the subsytem is present in the json subsystems file
        if (m_factory.isPresent())
        {
            SK25SwerveFactory SwerveFactory = m_factory.get();

            SwerveCommand swerveCommand = new SwerveCommand(
            SwerveFactory, 
            () -> kTranslationXPort.getFilteredAxis(), 
            () -> kTranslationYPort.getFilteredAxis(), 
            () -> kVelocityOmegaPort.getFilteredAxis()
            );

            //make the swerveCommand run when no other command which utilizes the SK25Swerve subsytem runs.
            SwerveFactory.setDefaultCommand(swerveCommand);
        }
    }

}

