package frc.robot.bindings;

import java.util.Optional;

import frc.robot.commands.SwerveCommand;
import frc.robot.subsystems.SK25Swerve;
import static frc.robot.Ports.DriverPorts.*;

public class SK25SwerveBinder implements CommandBinder{
    
    //create a subsystem toggleable by the json subsystems file
    Optional <SK25Swerve> m_swerve;


    public SK25SwerveBinder(Optional<SK25Swerve> m_swerve)
    {
        this.m_swerve = m_swerve;
    }
     
    public void bindButtons()
    {
        //if the subsytem is present in the json subsystems file
        if (m_swerve.isPresent())
        {
            SK25Swerve Swerve = m_swerve.get();

            SwerveCommand swerveCommand = new SwerveCommand(
            Swerve, 
            () -> kTranslationXPort.getFilteredAxis(), 
            () -> kTranslationYPort.getFilteredAxis(), 
            () -> kVelocityOmegaPort.getFilteredAxis()
            );

            //make the swerveCommand run when no other command which utilizes the SK25Swerve subsytem runs.
            Swerve.setDefaultCommand(swerveCommand);
        }
    }

}

