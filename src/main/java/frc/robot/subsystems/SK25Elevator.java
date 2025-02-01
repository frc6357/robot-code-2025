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
import com.ctre.phoenix.sensors.CANCoder;
import com.ctre.phoenix.sensors.CANCoderConfiguration;
import com.ctre.phoenix.sensors.SensorInitializationStrategy;

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
    SparkFlexConfig config1;
    SparkFlexConfig config2;

    //Create Memory PID Objects
    PIDController rPID;
    PIDController lPID;

    // Target & Current Position
    double LtargetPosition;
    double LcurrentPosition;
    double RtargetPosition;
    double RcurrentPosition;

    // Phoenix Encoder Objects
    CANCoder CANCoderL;
    CANCoder CANCoderR;

    // Encoder Objects
    RelativeEncoder encoderL;
    RelativeEncoder encoderR;

    // Touch Sensor Objects
    DigitalInput touchSensorTop;
    DigitalInput touchSensorBottom;

    // Magnetic Encoder Objects
    DigitalInput magEncoder1;
    DigitalInput magEncoder2;
    DigitalInput magEncoder3;
    DigitalInput magEncoder4;

    // Constructor For Public Command Access
    public SK25Elevator()
    {
        // Touch Sensors
        touchSensorTop = new DigitalInput(1);
        touchSensorBottom = new DigitalInput(2);

        // PID Controllers - Setpoints
        rPID = new PIDController(rightElevator.kP, rightElevator.kI, rightElevator.kD);
        lPID = new PIDController(leftElevator.kP, leftElevator.kI, leftElevator.kD);

        rPID.setSetpoint(0.0);
        lPID.setSetpoint(0.0);

        // Encoder Objects
        encoderL = motorL.getEncoder();
        encoderR = motorR.getEncoder();

        // Motor Initialization With REV Sparkflex - Configurations
        motorR = new SparkFlex(kRightElevatorMotor.ID, MotorType.kBrushless);
        motorL = new SparkFlex(kLeftElevatorMotor.ID, MotorType.kBrushless);
        config1 = new SparkFlexConfig();

        // Configurations For The Motors & Encoders
        config1
            .inverted(true)
            .idleMode(IdleMode.kBrake)
            .smartCurrentLimit(kElevatorCurrentLimit);

        config2
            .idleMode(IdleMode.kBrake)
            .smartCurrentLimit(kElevatorCurrentLimit);

        config1.encoder
            .positionConversionFactor(elevatorConversion);

        config2.encoder
            .positionConversionFactor(elevatorConversion);
        
        motorR.configure(config1, SparkBase.ResetMode.kResetSafeParameters, SparkBase.PersistMode.kNoPersistParameters);
        motorL.configure(config2, SparkBase.ResetMode.kResetSafeParameters, SparkBase.PersistMode.kNoPersistParameters);

        // Current, Target, and Reset Positions
        RtargetPosition = 0.0;
        RcurrentPosition = 0.0;

        LtargetPosition = 0.0;
        LcurrentPosition = 0.0;

        resetPosition();

        // CANCoder Stuff

        CANCoderL = new CANCoder(kEncoderL.ID, kEncoderL.bus);
        CANCoderConfiguration encoderConfigL = new CANCoderConfiguration();

        encoderConfigL.initializationStrategy = SensorInitializationStrategy.BootToZero;
        encoderConfigL.unitString = "deg";
        encoderConfigL.sensorDirection = false; // CCW+
        encoderConfigL.sensorCoefficient = 360.0 / 4096 / kCANCoderGearRatio;

        CANCoderL.configAllSettings(encoderConfigL);
        CANCoderL.setPosition(0.0);



        CANCoderR = new CANCoder(kEncoderR.ID, kEncoderR.bus);
        CANCoderConfiguration encoderConfigR = new CANCoderConfiguration();

        encoderConfigR.initializationStrategy = SensorInitializationStrategy.BootToZero;
        encoderConfigR.unitString = "deg";
        encoderConfigR.sensorDirection = false; // CCW+
        encoderConfigR.sensorCoefficient = 360.0 / 4096 / kCANCoderGearRatio;

        CANCoderR.configAllSettings(encoderConfigR);
        CANCoderR.setPosition(0.0);
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
        RtargetPosition = height;
        rPID.setSetpoint(RtargetPosition);
    }
    
    /**
     * {@inheritDoc}
     */
    public void setLeftTargetHeight(double height)
    {
        LtargetPosition = height;
        lPID.setSetpoint(LtargetPosition);
    }

    /**
     * {@inheritDoc}
     */
    public double getLeftPosition()
    {
        return CANCoderL.getPosition();
    }
    
    /**
     * {@inheritDoc}
     */
    public double getRightPosition()
    {
        return CANCoderR.getPosition();
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
    




    // Reset Position
    public void resetPosition()
    {
        encoderL.setPosition(0.0);
        encoderR.setPosition(0.0);
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

    // Run Motors Methods
    public void runLeftMotor(double speed)
    {
        motorL.set(speed);
    }

    public void runRightMotor(double speed)
    {
        motorR.set(speed);
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