package frc.robot.subsystems;

import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import static frc.robot.Konstants.ElevatorConstants.*;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.sim.SparkFlexExternalEncoderSim;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import static frc.robot.Ports.ElevatorPorts.*;

public class SK25Elevator extends SubsystemBase
{
    //Create memory motor objects
    SparkFlex motorR;
    SparkMax motorL;

    SparkFlexExternalEncoderSim encoderTestRight;

    //SparkBase motortest;

    SparkBaseConfig motortestconfig;

    //Create memory PID object
    PIDController rPID;
    PIDController lPID;

    double LtargetPosition;
    double LcurrentPosition;

    double RtargetPosition;
    double RcurrentPosition;

    RelativeEncoder encoderL;
    RelativeEncoder encoderR;

    DigitalInput touchSensorTop;
    DigitalInput touchSensorBottom;

    //Constructor for public command access
    public SK25Elevator()
    {
        //Initialize motor objects
        rPID = new PIDController(rightElevator.kP, rightElevator.kI, rightElevator.kD);
        rPID.setSetpoint(0.0);

        lPID = new PIDController(leftElevator.kP, leftElevator.kI, leftElevator.kD);
        lPID.setSetpoint(0.0);

        motorR = new SparkFlex(kRightElevatorMotor.ID, MotorType.kBrushless);
        motorL = new SparkMax(kLeftElevatorMotor.ID, MotorType.kBrushless);

        encoderTestRight = new SparkFlexExternalEncoderSim(motorR);

        // Motor configurations for inverted

        motortestconfig.inverted(true);
        motorR.configure(motortestconfig, SparkBase.ResetMode.kResetSafeParameters, SparkBase.PersistMode.kNoPersistParameters);

        // Encoder objects

        encoderL = motorL.getEncoder();
        encoderR = motorR.getEncoder();

        RelativeEncoder encoderR = motorR.getEncoder();
        encoderTestRight.setPositionConversionFactor(elevatorConversion);

        RelativeEncoder encoderL = motorL.getEncoder();
        //encoderL.setPositionConversionFactor(elevatorConversion);

        resetPosition(0.0);

        // Positions

        RtargetPosition = 0.0;
        RcurrentPosition = 0.0;

        LtargetPosition = 0.0;
        LcurrentPosition = 0.0;

        // Touch Sensor

        touchSensorTop = new DigitalInput(1);
        touchSensorBottom = new DigitalInput(2);
    }

    // Button Sensor Methods

    public Boolean isTopSensorPressed()
    {
        return !touchSensorTop.get();
    }

    public Boolean isBottomSensorPressed()
    {
        return !touchSensorBottom.get();
    }

    public boolean atTop()
    {
        if(isTopSensorPressed())
            return true;
        else
            return false;
    }

    public boolean atBottom()
    {
        if(isBottomSensorPressed())
            return true;
        else
            return false;
    }

    // Motor Methods

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

    // Position Methods
    
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

    // Stop Motors Method

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

        //TODO Uncomment below and add this to elastic dashboard once it's implemented.

        //SmartDashboard.putBoolean("Elevator At Top", atTop());
        //SmartDashboard.putBoolean("Elevator At Bottom", atBottom());
    }
}