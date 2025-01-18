package frc.robot.commands;

import static edu.wpi.first.wpilibj.XboxController.Axis.kLeftX;
import static edu.wpi.first.wpilibj.XboxController.Axis.kLeftY;
import static edu.wpi.first.wpilibj.XboxController.Axis.kRightX;
import static frc.robot.Ports.DriverPorts.swerveController;

import edu.wpi.first.wpilibj2.command.Command;
//import com.ctre.phoenix6.hardware.core.CorePigeon2;
import frc.robot.subsystems.NewSwerve;


public class SwerveCommand extends Command
{
    @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
    private final NewSwerve swerve;

    public SwerveCommand(NewSwerve _swerve)
    {
        swerve = _swerve;
        addRequirements(swerve);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() 
    {
      
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() 
    {
        //driverController conversion
        double angle = Math.atan2(swerveController.getRawAxis(kLeftY.value), swerveController.getRawAxis(kLeftX.value));
        //make the deadzones
        double translationMagnitude = deadZone(Math.hypot(swerveController.getRawAxis(kLeftX.value), swerveController.getRawAxis(kLeftY.value)), 0.1);
        double rotationMagnitude = deadZone(swerveController.getRawAxis(kRightX.value), 0.1);

        //feild centric controls
        //TODO: getAngle() is deprecated for removal in 2026, use getYaw() from CorePigeon2 class instead and convert to degrees.
        angle -= NewSwerve.pigeon.getAngle();

        NewSwerve.manager.setSwerve(angle, translationMagnitude, rotationMagnitude);
    }
    

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) 
    {
       //sets all wheels to zero
       NewSwerve.manager.setSwerve(0.0, 0.0, 0.0);
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished()
    {
        return false;
    }

    private double deadZone (double value, double deadZone)
    {
        if( Math.abs(value) < deadZone)
        {
            return 0;
        }
        return value;
    }
    
    public void initDeafultCommand()
    {
        swerve.setDefaultCommand(new SwerveCommand(swerve));
    }
}
