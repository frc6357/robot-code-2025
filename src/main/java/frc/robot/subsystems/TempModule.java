package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.subsystems.swerve.SwerveConstantsConfigurator;

public class TempModule {
    
    //the contsantsConfig class will handle logic with the motors, encoders, and most wpilib classes

    //rotation controller
    //translation 2d of the module
    //velocity limiter


    //constructor (configs)
    public TempModule(SwerveConstantsConfigurator config)
    {
        
    }

    //new velocity limiter
    //new rotation controller
    //new module translation


    //getters

    //getModuleTranslation
    //getModuleRotation
    //getModuleState
    //getPosition
    public Transform2d m = new Transform2d(new Translation2d(), new Rotation2d());


    //setters

    //setModuleState


    //others

    //state.optimize
    //state.decreaseError
    //apply PID

}
