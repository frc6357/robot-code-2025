// Essentials
package frc.robot.subsystems;

import edu.wpi.first.math.MathUtil;
import frc.robot.subsystems.superclasses.Elevator;

// Constants (Muy Importante)
import static frc.robot.Konstants.ElevatorConstants.*;
//import static frc.robot.Konstants.ElevatorConstants.kElevatorCurrentLimit;
//import static frc.robot.Konstants.ElevatorConstants.elevatorConversion;
//import static frc.robot.Konstants.ElevatorConstants.kPositionTolerance;
//import static frc.robot.Konstants.ElevatorConstants.leftElevator;
//import static frc.robot.Konstants.ElevatorConstants.rightElevator;
//import static frc.robot.Konstants.ElevatorConstants.kElevatorMotorMaxOutput;
//import static frc.robot.Konstants.ElevatorConstants.kElevatorMotorMinOutput;
//import static frc.robot.Konstants.ElevatorConstants.kCANCoderGearRatio;
import static frc.robot.Ports.ElevatorPorts.kLeftElevatorMotor;
import static frc.robot.Ports.ElevatorPorts.kRightElevatorMotor;
import static frc.robot.Ports.ElevatorPorts.kEncoderL;
import static frc.robot.Ports.ElevatorPorts.kEncoderR;

// Phoenix Sensors (Help)
import com.ctre.phoenix6.hardware.CANcoder;
//import com.ctre.phoenix.sensors.CANCoder;
//import com.ctre.phoenix.sensors.CANCoderConfiguration;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import com.ctre.phoenix6.StatusSignal;

// Encoders - Sensors
import com.revrobotics.RelativeEncoder;

// SparkBase
import com.revrobotics.spark.SparkBase;

// Motors - Sparkflex
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
// Configurations For Stuff (Thanks REV)
import com.revrobotics.spark.config.SparkFlexConfig;

// PID Controller
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.DigitalInput;

// SmartDashboard
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Konstants.ElevatorConstants.ElevatorPosition;

// Unused Imports (Maybe In The Future)
//import com.revrobotics.spark.config.SparkMaxConfig;
//import com.revrobotics.spark.config.SparkBaseConfig;
//import com.revrobotics.spark.SparkRelativeEncoder;
//import com.revrobotics.spark.SparkMax;

public class SK25Elevator extends Elevator
{
    // Create Memory Motor Objects
    SparkFlex motorR;
    SparkFlex motorL;

    // Creating Config Object
    SparkFlexConfig motorConfigL;
    SparkFlexConfig motorConfigR;

    //Create Memory PID Objects
    PIDController rPID;
    PIDController lPID;

    // Target & Current Position
    double LtargetHeight;
    double LcurrentHeight;
    double RtargetHeight;
    double RcurrentHeight;

    // CANcoder Objects
    CANcoder CANCoderL;
    CANcoder CANCoderR;
    CANcoderConfiguration CANCoderConfigL;
    CANcoderConfiguration CANCoderConfigR;
    StatusSignal<Angle> CANcoderDegreesL;
    StatusSignal<Angle> CANcoderDegreesR;
    double CANcoderHeightL;
    double CANcoderHeightR;

    // Encoder Objects
    //RelativeEncoder encoderL;
    //RelativeEncoder encoderR;

    // Touch Sensor Objects
    DigitalInput touchSensorTop;
    DigitalInput touchSensorBottom;

    // Magnetic Encoder Objects
    DigitalInput magEncoder1;
    DigitalInput magEncoder2;
    DigitalInput magEncoder3;
    DigitalInput magEncoder4;

    SlewRateLimiter accelLimit;

    // Constructor For Public Command Access
    public SK25Elevator()
    {
        // Touch Sensors
        //touchSensorTop = new DigitalInput(1);
        //touchSensorBottom = new DigitalInput(2);

        // PID Controllers - Setpoints
        rPID = new PIDController(rightElevator.kP, rightElevator.kI, rightElevator.kD);
        lPID = new PIDController(leftElevator.kP, leftElevator.kI, leftElevator.kD);

        rPID.setIntegratorRange(kMinInteg, kMaxInteg);
        lPID.setIntegratorRange(kMinInteg, kMaxInteg);

        rPID.setSetpoint(0.0);
        lPID.setSetpoint(0.0);

        // Encoder Objects
        //encoderL = motorL.getEncoder();
        //encoderR = motorR.getEncoder();

        // Motor Initialization With REV Sparkflex - Configurations
        motorR = new SparkFlex(kRightElevatorMotor.ID, MotorType.kBrushless);
        motorL = new SparkFlex(kLeftElevatorMotor.ID, MotorType.kBrushless);
        motorConfigL = new SparkFlexConfig();
        motorConfigR = new SparkFlexConfig();
        
        // Rate Limiter
        accelLimit = new SlewRateLimiter(kPositiveAccelLimit, kNegativeAccelLimit, 0.0);

        // Configurations For The Motors & Encoders
        motorConfigL
            .inverted(true)
            .idleMode(IdleMode.kBrake)
            .smartCurrentLimit(kElevatorCurrentLimit);

        motorConfigR
            .idleMode(IdleMode.kBrake)
            .smartCurrentLimit(kElevatorCurrentLimit);

        //motorConfigL.encoder
            //.positionConversionFactor(elevatorConversion);

        //motorConfigR.encoder
            //.positionConversionFactor(elevatorConversion);
        
        motorR.configure(motorConfigL, SparkBase.ResetMode.kResetSafeParameters, SparkBase.PersistMode.kNoPersistParameters);
        motorL.configure(motorConfigR, SparkBase.ResetMode.kResetSafeParameters, SparkBase.PersistMode.kNoPersistParameters);

        // Current, Target, and Reset Positions
        RtargetHeight = 0.0;
        RcurrentHeight = 0.0;

        LtargetHeight = 0.0;
        LcurrentHeight = 0.0;

        //resetPosition();
        
        //TODO FIX_BEFORE_TESTING - Verify motor inverted (clockwise/ccw)
        // CANCoders
        CANcoder CANCoderL = new CANcoder(kEncoderL.ID, kEncoderL.bus);
        CANcoderConfiguration CANCoderConfigL = new CANcoderConfiguration();
        CANCoderConfigL.MagnetSensor.SensorDirection = SensorDirectionValue.CounterClockwise_Positive;
        CANCoderL.getConfigurator().apply(CANCoderConfigL);
        CANCoderL.setPosition(0.0);

        CANcoder CANCoderR = new CANcoder(kEncoderR.ID, kEncoderR.bus);
        CANcoderConfiguration CANCoderConfigR = new CANcoderConfiguration();
        CANCoderConfigR.MagnetSensor.SensorDirection = SensorDirectionValue.Clockwise_Positive;
        CANCoderR.getConfigurator().apply(CANCoderConfigR);
        // Set the position to 0 rotations for initial use
        CANCoderR.setPosition(0.0); 


        // Old Phoenix 5 Code
        // In Phoenix 6, CANcoder does not support setting a custom sensor coefficient, unit string, 
        // and sensor time base. Instead, the CANcoder uses canonical units of rotations and rotations per second using the C++ units library.
        //encoderConfigL.initializationStrategy = SensorInitializationStrategy.BootToZero; - Always boots to 0 in Phoenix 6
        //CANCoderConfigL.unitString = "deg";
        //CANCoderConfigL.sensorDirection = false; // CCW+ 
        //CANCoderConfigL.sensorCoefficient = 360.0 / 4096 / kCANCoderGearRatio;
        //CANCoderL.configAllSettings(CANCoderConfigL);
    }


    /* 
     *  KURIAN-RELATED METHODS 
    */

    
    /**
     * {@inheritDoc}
     */
    public void setTargetHeight(ElevatorPosition height)
    {
        setRightTargetHeight(height.height);
        setLeftTargetHeight(height.height);
    }

    /**
     * {@inheritDoc}
     */
    public void setRightTargetHeight(double height)
    {
        RtargetHeight = height;
        rPID.setSetpoint(RtargetHeight);
    }
    
    /**
     * {@inheritDoc}
     */
    public void setLeftTargetHeight(double height)
    {
        LtargetHeight = height;
        lPID.setSetpoint(LtargetHeight);
    }

    /**
     * {@inheritDoc}
     */
    public double getLeftPosition()
    {
        // Get angle from CANcoder
        CANcoderDegreesL = CANCoderL.getPosition();

        // Convert degrees to inches
        CANcoderHeightL = 1;
        
        return CANcoderHeightL;
    }
    
    /**
     * {@inheritDoc}
     */
    public double getRightPosition()
    {
        // Get angle from CANcoder
        CANcoderDegreesR = CANCoderR.getPosition();

        // Convert degrees to inches
        CANcoderHeightR = 1;
        
        return CANcoderHeightR;
    }





    public double getRightTargetPosition(){
        return RtargetHeight;
    }

    public double getLeftTargetPosition(){
        return LtargetHeight;
    }





    public boolean isRightAtTargetPosition()
    {
        return Math.abs(getRightPosition() - getRightTargetPosition()) < kPositionTolerance;
    }

    public boolean isLeftAtTargetPosition()
    {
        return Math.abs(getLeftPosition() - getLeftTargetPosition()) < kPositionTolerance;
    }
    




    // Reset Position
    public void resetPosition()
    {
        CANCoderL.setPosition(0.0);
        CANCoderR.setPosition(0.0);
    }


    /* 
     *  NON-KURIAN METHODS 
    */


    // Button Sensor Methods
    public Boolean isTopSensorPressed()
    {
        return !touchSensorTop.get();
    }

    public Boolean isBottomSensorPressed()
    {
        return !touchSensorBottom.get();
    }

    // Button Sensor Methods For Use By Commands
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

    public void runLeftMotor(double motorSpeed)
    {
        motorL.set(motorSpeed);
    }

    public void runRightMotor(double motorSpeed)
    {
        motorR.set(motorSpeed);
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
        double r_target_position = getRightTargetPosition();

        double l_current_position = getLeftPosition();
        double l_target_position = getLeftTargetPosition();

        // Calculates motor speed and puts it within operating range
        double rSpeed = MathUtil.clamp(rPID.calculate(r_current_position), kElevatorMotorMinOutput, kElevatorMotorMaxOutput);
        motorR.set(rSpeed); 

        // Calculates motor speed and puts it within operating range
        double lSpeed = MathUtil.clamp(lPID.calculate(l_current_position), kElevatorMotorMinOutput, kElevatorMotorMaxOutput);
        motorL.set(lSpeed); 

        SmartDashboard.putNumber("Right Current Position", r_current_position);
        SmartDashboard.putNumber("Right Target Position", r_target_position);
        SmartDashboard.putBoolean("Right Elevator at Setpoint", isRightAtTargetPosition());

        SmartDashboard.putNumber("Left Current Position", l_current_position);
        SmartDashboard.putNumber("Left Target Position", l_target_position);
        SmartDashboard.putBoolean("Left Elevator at Setpoint", isLeftAtTargetPosition());

        //TODO Uncomment below and add this to elastic dashboard once it's implemented.

        //SmartDashboard.putBoolean("Elevator At Top", atTop());
        //SmartDashboard.putBoolean("Elevator At Bottom", atBottom());
    }
}