package frc.robot.subsystems;

import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import static frc.robot.Konstants.ElevatorConstants.*;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import static frc.robot.Ports.ElevatorPorts.*;

public class SK25Elevator extends SubsystemBase
{
    //Create memory motor objects
    SparkFlex motorR;
    SparkFlex motorL;
    
    //Create memory PID object
    PIDController rPID;
    PIDController lPID;

    double LtargetPosition;
    double LcurrentPosition;

    double RtargetPosition;
    double RcurrentPosition;

    RelativeEncoder encoderL;
    RelativeEncoder encoderR;

    //Constructor for public command access
    public SK25Elevator()
    {
        //Initialize motor objects
        rPID = new PIDController(rightElevator.kP, rightElevator.kI, rightElevator.kD);
        rPID.setSetpoint(0.0);

        lPID = new PIDController(leftElevator.kP, leftElevator.kI, leftElevator.kD);
        lPID.setSetpoint(0.0);

        motorR = new SparkFlex(kRightClimbMotor.ID, MotorType.kBrushless);
        motorL = new SparkFlex(kLeftClimbMotor.ID, MotorType.kBrushless);

        motorL.setInverted(false);
        motorR.setInverted(true);

        encoderL = motorL.getEncoder();
        encoderR = motorR.getEncoder();
        
        RelativeEncoder encoderR = motorR.getEncoder();
        //encoderR.setPositionConversionFactor(elevatorConversion);

        RelativeEncoder encoderL = motorL.getEncoder();
        //encoderL.setPositionConversionFactor(elevatorConversion);
        resetPosition(0.0);

        RtargetPosition = 0.0;
        RcurrentPosition = 0.0;

        LtargetPosition = 0.0;
        LcurrentPosition = 0.0;
    }

    public void setRightMotor(double location)
    {
        RtargetPosition = location;
        rPID.setSetpoint(location);
    }
    
    public void setLeftMotor(double location)
    {
        LtargetPosition = location;
        lPID.setSetpoint(location);
    }

    public void runLeftMotor(double speed)
    {
        motorL.set(speed);
    }

    public void runRightMotor(double speed)
    {
        motorR.set(speed);
    }
    public double getLeftPosition()
    {
        return encoderL.getPosition();
    }
    
    public double getRightPosition()
    {
        return encoderR.getPosition();
    }

    public double getRightTargetPosition(){
        return RtargetPosition;
    }

    public double getLeftTargetPosition(){
        return LtargetPosition;
    }

    public boolean isRightAtTargetPosition()
    {
        return Math.abs(getRightPosition() - getRightTargetPosition()) < kPositionTolerance;
    }

    public boolean isLeftAtTargetPosition()
    {
        return Math.abs(getLeftPosition() - getLeftTargetPosition()) < kPositionTolerance;
    }

    public void resetPosition(double position){
        encoderL.setPosition(position);
        encoderR.setPosition(position);
    }

    public void stopMotors()
    {
        motorL.stopMotor();
        motorR.stopMotor();
    }

    @Override
    public void periodic(){
        
        double r_current_position = getRightPosition();
        // double r_target_position = getRightTargetPosition();

        double l_current_position = getLeftPosition();
        // double l_target_position = getLeftTargetPosition();

        // // Calculates motor speed and puts it within operating range
        // double rSpeed = MathUtil.clamp(rPID.calculate(r_current_position), kClimbMotorMinOutput, kClimbMotorMaxOutput);
        // motorR.set(rSpeed); 

        // // Calculates motor speed and puts it within operating range
        // double lSpeed = MathUtil.clamp(lPID.calculate(l_current_position), kClimbMotorMinOutput, kClimbMotorMaxOutput);
        // motorL.set(lSpeed); 

        SmartDashboard.putNumber("Right Current Position", r_current_position);
        // SmartDashboard.putNumber("Right Target Position", r_target_position);
        // SmartDashboard.putBoolean("Right Arm at Setpoint", isRightAtTargetPosition());

        SmartDashboard.putNumber("Left Current Position", l_current_position);
        // SmartDashboard.putNumber("Left Target Position", l_target_position);
        // SmartDashboard.putBoolean("Left Arm at Setpoint", isLeftAtTargetPosition());
    }
}