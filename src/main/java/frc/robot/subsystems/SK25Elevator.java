// Essentials
package frc.robot.subsystems;
import edu.wpi.first.math.MathUtil;
import frc.robot.subsystems.superclasses.Elevator;

// Constants (Muy Importante)
import static frc.robot.Konstants.ElevatorConstants.*;
import static frc.robot.Ports.ElevatorPorts.*;

// Motors - Sparkflex
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;

// Configurations For Motors (REV)
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

// PID Controller
import edu.wpi.first.math.controller.PIDController;

// Limit Switches
import edu.wpi.first.wpilibj.DigitalInput;

// Absolute Encoder (REV)
import edu.wpi.first.wpilibj.DutyCycleEncoder;

// SmartDashboard
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class SK25Elevator extends Elevator
{
    // Create Memory Motor Objects
    SparkFlex motorR;
    SparkFlex motorL;
    
    // Encoder Memory Object
    DutyCycleEncoder absoluteEncoder;

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

    // Touch Sensor Objects
    DigitalInput touchSensorTop;
    DigitalInput touchSensorBottom;

    // Constructor For Public Command Access
    public SK25Elevator()
    {
        // PID Controllers - Setpoints
        rPID = new PIDController(rightElevator.kP, rightElevator.kI, rightElevator.kD);
        lPID = new PIDController(leftElevator.kP, leftElevator.kI, leftElevator.kD);

        rPID.setIntegratorRange(kMinInteg, kMaxInteg);
        lPID.setIntegratorRange(kMinInteg, kMaxInteg);

        rPID.setSetpoint(0.0);
        lPID.setSetpoint(0.0);

        // Motor Initialization With REV Sparkflex - Configurations
        motorR = new SparkFlex(kRightElevatorMotor.ID, MotorType.kBrushless);
        motorL = new SparkFlex(kLeftElevatorMotor.ID, MotorType.kBrushless);

        motorConfigL = new SparkFlexConfig();
        motorConfigR = new SparkFlexConfig();
        
        // TODO Change the channel, full range, and expected zero.
        // Initializes a duty cycle encoder on DIO pins 0 to return a value of 4 for
        // a full rotation, with the encoder reporting 0 half way through rotation (2
        // out of 4)
        absoluteEncoder = new DutyCycleEncoder(0, 4.0, 2.0);

        // Configurations For The Motors & Encoders
        motorConfigL
            .idleMode(IdleMode.kBrake)
            .smartCurrentLimit(kElevatorCurrentLimit);

        motorConfigR
            .inverted(true)
            .idleMode(IdleMode.kBrake)
            .smartCurrentLimit(kElevatorCurrentLimit);
        
        // Apply Motor Configurations
        motorR.configure(motorConfigR, SparkBase.ResetMode.kResetSafeParameters, SparkBase.PersistMode.kNoPersistParameters);
        motorL.configure(motorConfigL, SparkBase.ResetMode.kResetSafeParameters, SparkBase.PersistMode.kNoPersistParameters);

        // Current, Target, and Reset Positions
        RtargetHeight = 0.0;
        RcurrentHeight = 0.0;

        LtargetHeight = 0.0;
        LcurrentHeight = 0.0;

        // Touch Sensors
        // touchSensorTop = new DigitalInput(1);
        // touchSensorBottom = new DigitalInput(2);
    }

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
    public double getEncoderPosition()
    {
        double encoderPositionValue = absoluteEncoder.get();
        return encoderPositionValue;

    }

    /**
     * {@inheritDoc}
     */
    public double getRightTargetPosition()
    {
        return RtargetHeight;
    }

    /**
     * {@inheritDoc}
     */
    public double getLeftTargetPosition()
    {
        return LtargetHeight;
    }

    /**
     * {@inheritDoc}
     */
    public boolean isRightAtTargetPosition()
    {
        return Math.abs(getEncoderPosition() - getRightTargetPosition()) < kPositionTolerance;
    }

    /**
     * {@inheritDoc}
     */
    public boolean isLeftAtTargetPosition()
    {
        return Math.abs(getEncoderPosition() - getLeftTargetPosition()) < kPositionTolerance;
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

    public void stopMotors()
    {
        motorL.stopMotor();
        motorR.stopMotor();
    }

    @Override
    public void periodic()
    {
        // Initialize Current & Target Positions
        double currentPosition = getEncoderPosition();

        double rTargetPosition = getRightTargetPosition();
        double lTargetPosition = getLeftTargetPosition();

        // Calculates Motor Speed & Puts It Within Operating Range
        double rSpeed = MathUtil.clamp(rPID.calculate(currentPosition), kElevatorMotorMinOutput, kElevatorMotorMaxOutput);
        motorR.set(rSpeed); 

        double lSpeed = MathUtil.clamp(lPID.calculate(currentPosition), kElevatorMotorMinOutput, kElevatorMotorMaxOutput);
        motorL.set(lSpeed); 

        // SmartDashboard Current & Target Positions
        SmartDashboard.putNumber("Current Estimated Position", currentPosition);

        SmartDashboard.putNumber("Right Target Position", rTargetPosition);
        SmartDashboard.putBoolean("Right Elevator at Setpoint", isRightAtTargetPosition());

        SmartDashboard.putNumber("Left Target Position", lTargetPosition);
        SmartDashboard.putBoolean("Left Elevator at Setpoint", isLeftAtTargetPosition());
        
        //TODO Uncomment below and add this to elastic dashboard once it's implemented.
        //SmartDashboard.putBoolean("Elevator At Top", atTop());
        //SmartDashboard.putBoolean("Elevator At Bottom", atBottom());
    }
    public void testPeriodic(){}
    public void testInit(){}
}