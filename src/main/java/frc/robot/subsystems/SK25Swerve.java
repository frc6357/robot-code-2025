package frc.robot.subsystems;

import static frc.robot.Konstants.SwerveConstants.kBackLeftDriveMotorID;
import static frc.robot.Konstants.SwerveConstants.kBackLeftEncoderID;
import static frc.robot.Konstants.SwerveConstants.kBackLeftEncoderOffsetDouble;
import static frc.robot.Konstants.SwerveConstants.kBackLeftTurnMotorID;
import static frc.robot.Konstants.SwerveConstants.kBackRightDriveMotorID;
import static frc.robot.Konstants.SwerveConstants.kBackRightEncoderID;
import static frc.robot.Konstants.SwerveConstants.kBackRightEncoderOffsetDouble;
import static frc.robot.Konstants.SwerveConstants.kBackRightTurnMotorID;
import static frc.robot.Konstants.SwerveConstants.kFrontLeftDriveMotorID;
import static frc.robot.Konstants.SwerveConstants.kFrontLeftEncoderID;
import static frc.robot.Konstants.SwerveConstants.kFrontLeftEncoderOffsetDouble;
import static frc.robot.Konstants.SwerveConstants.kFrontLeftTurnMotorID;
import static frc.robot.Konstants.SwerveConstants.kFrontRightDriveMotorID;
import static frc.robot.Konstants.SwerveConstants.kFrontRightEncoderID;
import static frc.robot.Konstants.SwerveConstants.kFrontRightEncoderOffsetDouble;
import static frc.robot.Konstants.SwerveConstants.kFrontRightTurnMotorID;

import com.ctre.phoenix6.hardware.Pigeon2;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class SK25Swerve extends SubsystemBase{

    public static SK25SwerveFactory factory;

    public static Pigeon2 pigeon;

    SK25SwerveModule fLModule;
    SK25SwerveModule fRModule;
    SK25SwerveModule bLModule;
    SK25SwerveModule bRModule;

    public SK25Swerve()
    {
        fLModule = new SK25SwerveModule(kFrontLeftDriveMotorID, kFrontLeftTurnMotorID, kFrontLeftEncoderID, kFrontLeftEncoderOffsetDouble);
        fRModule = new SK25SwerveModule(kFrontRightDriveMotorID, kFrontRightTurnMotorID, kFrontRightEncoderID, kFrontRightEncoderOffsetDouble);
        bLModule = new SK25SwerveModule(kBackLeftDriveMotorID, kBackLeftTurnMotorID, kBackLeftEncoderID, kBackLeftEncoderOffsetDouble);
        bRModule = new SK25SwerveModule(kBackRightDriveMotorID, kBackRightTurnMotorID, kBackRightEncoderID, kBackRightEncoderOffsetDouble);

        factory = new SK25SwerveFactory(fLModule, fRModule, bLModule, bRModule, pigeon);
    }

    //occurs every 20 miliseconds, usually not tied to a command, binder, etc...
    public void periodic()
    {
    } 

    public void testInit()
    {
    }
    
    public void testPeriodic()
    {
    }
}
