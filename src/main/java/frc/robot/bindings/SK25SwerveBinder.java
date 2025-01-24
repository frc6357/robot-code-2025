package frc.robot.bindings;

import java.util.Optional;

import frc.robot.commands.SwerveCommand;
import frc.robot.subsystems.SK25Swerve;
import static frc.robot.Ports.DriverPorts.*;

public class SK25SwerveBinder implements CommandBinder{
    
    //create a subsystem toggleable by the json subsystems file
    Optional <SK25Swerve> m_swerve;

    //create the ExampleButton trigger object 


    double leftX = kTranslationXPort.getFilteredAxis();
    double leftY = kTranslationYPort.getFilteredAxis();
    double rightX = kVelocityOmegaPort.getFilteredAxis();

    public SK25SwerveBinder(Optional<SK25Swerve> swerve)
    {
        this.m_swerve = swerve;

    }
     
    public void bindButtons()
    {
        //if the subsytem is present in the json subsystems file
        if (m_swerve.isPresent())
        {
            SK25Swerve Swerve = m_swerve.get();

            SwerveCommand swerveCommand = new SwerveCommand(Swerve, leftX, leftY, rightX);
            Swerve.setDefaultCommand(swerveCommand);


        }
    }

}
