package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.swerve.SwerveDrivetrain;

import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.subsystems.swerve.SwerveConstantsConfigurator;

public class TempSwerve extends SwerveDrivetrain<TalonFX, TalonFX, CANcoder> implements Subsystem
{
    //stuff
    private SwerveConstantsConfigurator config = new SwerveConstantsConfigurator();


    //constructor
    public TempSwerve()
    {
        super(new TalonFX::new TalonFX::new CANcoder, config.getDrivetrainConstants(), config.getModules());)
    }
}
