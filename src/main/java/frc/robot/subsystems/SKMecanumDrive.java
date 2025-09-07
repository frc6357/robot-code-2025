package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj.drive.MecanumDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorController;
import edu.wpi.first.wpilibj.motorcontrol.PWMTalonFX;

public class SKMecanumDrive extends MecanumDrive {
    PWMTalonFX frontLeft;
    PWMTalonFX frontRight;
    PWMTalonFX backLeft;
    PWMTalonFX backRight;

    public SKMecanumDrive(
        TalonFX frontLeftMotor, 
        TalonFX frontRightMotor, 
        TalonFX backLeftMotor, 
        TalonFX backRightMotor)
    {
        frontLeft = new PWMTalonFX(frontLeftMotor.getDeviceID());
        frontRight = new PWMTalonFX(frontRightMotor.getDeviceID());
        backLeft = new PWMTalonFX(backLeftMotor.getDeviceID());
        backRight = new PWMTalonFX(backRightMotor.getDeviceID());


        super(frontLeft, frontRight, backLeft, backRight);
    }
}
