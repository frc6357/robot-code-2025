// Essentials
package frc.robot.subsystems;
// Constants (Muy Importante)
import static frc.robot.Konstants.ElevatorConstants.kElevatorCurrentLimit;
import static frc.robot.Konstants.ElevatorConstants.kElevatorMotorMaxOutput;
import static frc.robot.Konstants.ElevatorConstants.kElevatorMotorMinOutput;
import static frc.robot.Konstants.ElevatorConstants.kMaxInteg;
import static frc.robot.Konstants.ElevatorConstants.kMinInteg;
import static frc.robot.Konstants.ElevatorConstants.kPositionTolerance;
import static frc.robot.Konstants.ElevatorConstants.leftElevator;
import static frc.robot.Konstants.ElevatorConstants.rightElevator;
import static frc.robot.Ports.ElevatorPorts.kLeftElevatorMotor;
import static frc.robot.Ports.ElevatorPorts.kRightElevatorMotor;

// Encoder V3 (Still REV)
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.ClosedLoopSlot;
// Motors - Sparkflex
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
// Configurations For Motors (REV)
import com.revrobotics.spark.config.SparkFlexConfig;

import edu.wpi.first.math.MathUtil;
// PID Controller
import edu.wpi.first.math.controller.PIDController;
// Limit Switches
import edu.wpi.first.wpilibj.DigitalInput;
// Absolute Encoder (REV)
import edu.wpi.first.wpilibj.DutyCycleEncoder;
// SmartDashboard
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Konstants.ElevatorConstants.ElevatorPosition;
import frc.robot.subsystems.superclasses.Elevator;

public class SK25Elevator extends Elevator
{
    // Create Memory Motor Objects
    SparkFlex motorR;
    SparkFlex motorL;

    // REV CLOSED LOOP (V3)
    SparkClosedLoopController closedLoopController;
    RelativeEncoder encoder;
    
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
        // TODO Change the channel, full range, and expected zero.
        // Initializes a duty cycle encoder on DIO pins 0 to return a value of 4 for
        // a full rotation, with the encoder reporting 0 half way through rotation (2 out of 4)
        absoluteEncoder = new DutyCycleEncoder(0, 4.0, 2.0);

        // PID Controllers - Setpoints
        lPID = new PIDController(leftElevator.kP, leftElevator.kI, leftElevator.kD);
        rPID = new PIDController(rightElevator.kP, rightElevator.kI, rightElevator.kD);
        
        lPID.setIntegratorRange(kMinInteg, kMaxInteg);
        rPID.setIntegratorRange(kMinInteg, kMaxInteg);

        lPID.setSetpoint(0.0);
        rPID.setSetpoint(0.0);
        
        // Motor Initialization With REV Sparkflex - Configurations
        motorL = new SparkFlex(kLeftElevatorMotor.ID, MotorType.kBrushless);
        motorR = new SparkFlex(kRightElevatorMotor.ID, MotorType.kBrushless);

        
        
        motorConfigL = new SparkFlexConfig();
        motorConfigR = new SparkFlexConfig();

        // Configurations For The Left Motor & Encoder
        motorConfigL
            .idleMode(IdleMode.kBrake)
            .smartCurrentLimit(kElevatorCurrentLimit);

        /*
         *  ALL REV CLOSED LOOP ENCODER STUFF
         */

        motorConfigL.encoder
            .positionConversionFactor(1)
            .velocityConversionFactor(1);
        motorConfigL.closedLoop
            .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
            // Set PID values for position control. We don't need to pass a closed loop
            // slot, as it will default to slot 0.
            .p(0.1)
            .i(0)
            .d(0)
            .outputRange(-1, 1)
            // Set PID values for velocity control in slot 1
            .p(0.0001, ClosedLoopSlot.kSlot1)
            .i(0, ClosedLoopSlot.kSlot1)
            .d(0, ClosedLoopSlot.kSlot1)
            .velocityFF(1.0 / 5767, ClosedLoopSlot.kSlot1)
            .outputRange(-1, 1, ClosedLoopSlot.kSlot1);

        // Initialize dashboard values
        SmartDashboard.setDefaultNumber("Target Position", 0);
        SmartDashboard.setDefaultNumber("Target Velocity", 0);
        SmartDashboard.setDefaultBoolean("Control Mode", false);
        SmartDashboard.setDefaultBoolean("Reset Encoder", false);






        // Configurations For The Right Motor
        motorConfigR
            .inverted(true)
            .idleMode(IdleMode.kBrake)
            .smartCurrentLimit(kElevatorCurrentLimit);
        
        // Apply Motor Configurations
        motorR.configure(motorConfigR, SparkBase.ResetMode.kResetSafeParameters, SparkBase.PersistMode.kNoPersistParameters);
        motorL.configure(motorConfigL, SparkBase.ResetMode.kResetSafeParameters, SparkBase.PersistMode.kNoPersistParameters);

        // Encoder V3
        closedLoopController = motorL.getClosedLoopController();
        encoder = motorL.getEncoder();

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
        double encoderPositionValue = encoder.getPosition();
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

    @Override
    public void periodic()
    {

        /*
         *  ALL REV CLOSED LOOP ENCODER STUFF
         */

        if (SmartDashboard.getBoolean("Control Mode", false)) 
        {
            /*
            * Get the target velocity from SmartDashboard and set it as the setpoint
            * for the closed loop controller.
            */
            double targetVelocity = SmartDashboard.getNumber("Target Velocity", 0);
            closedLoopController.setReference(targetVelocity, ControlType.kVelocity, ClosedLoopSlot.kSlot1);
        } 
        else 
        {
            /*
            * Get the target position from SmartDashboard and set it as the setpoint
            * for the closed loop controller.
            */
            double targetPosition = SmartDashboard.getNumber("Target Position", 0);
            closedLoopController.setReference(targetPosition, ControlType.kPosition, ClosedLoopSlot.kSlot0);
        }

        // Display encoder position and velocity
        SmartDashboard.putNumber("Actual Position", encoder.getPosition());
        SmartDashboard.putNumber("Actual Velocity", encoder.getVelocity());

        if (SmartDashboard.getBoolean("Reset Encoder", false)) 
        {
            SmartDashboard.putBoolean("Reset Encoder", false);
            // Reset the encoder position to 0
            encoder.setPosition(0);
        }






        // Initialize Current & Target Positions
        /*
        double currentPosition = getEncoderPosition();
        double lTargetPosition = getLeftTargetPosition();
        double rTargetPosition = getRightTargetPosition();
        
        // Calculates Motor Speed & Puts It Within Operating Range
        double lSpeed = MathUtil.clamp(lPID.calculate(currentPosition), kElevatorMotorMinOutput, kElevatorMotorMaxOutput);
        double rSpeed = MathUtil.clamp(rPID.calculate(currentPosition), kElevatorMotorMinOutput, kElevatorMotorMaxOutput);

        motorL.set(lSpeed); 
        motorR.set(rSpeed); 
        
        // SmartDashboard Current & Target Positions
        SmartDashboard.putNumber("Current Estimated Position", currentPosition);

        SmartDashboard.putNumber("Left Target Position", lTargetPosition);
        SmartDashboard.putBoolean("Left Elevator at Setpoint", isLeftAtTargetPosition());

        SmartDashboard.putNumber("Right Target Position", rTargetPosition);
        SmartDashboard.putBoolean("Right Elevator at Setpoint", isRightAtTargetPosition());

        //TODO Uncomment below and add this to elastic dashboard once it's implemented.
        //SmartDashboard.putBoolean("Elevator At Top", atTop());
        //SmartDashboard.putBoolean("Elevator At Bottom", atBottom());
        */
    }
    public void testPeriodic(){}
    public void testInit(){}
}