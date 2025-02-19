// Essentials
package frc.robot.subsystems;
import edu.wpi.first.math.MathUtil;

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

// PID Controller
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.networktables.NTSendable;
import edu.wpi.first.networktables.NTSendableBuilder;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
// Limit Switches
import edu.wpi.first.wpilibj.DigitalInput;

// SmartDashboard
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.WPILibVersion;
import frc.robot.Konstants.ElevatorConstants.ElevatorPosition;
import frc.robot.preferences.Pref;
import frc.robot.preferences.SKPreferences;
import frc.robot.Robot;
import frc.robot.subsystems.superclasses.Elevator;
import com.revrobotics.spark.config.LimitSwitchConfig.Type;

import lombok.Getter;
import lombok.Setter;

public class SK25Elevator extends Elevator
{
    // Create Memory Motor Objects
    SparkFlex motorR;
    SparkFlex motorL;

    // REV CLOSED LOOP (V3)
    SparkClosedLoopController closedLoopController;
    RelativeEncoder encoder;

    // Creating Config Object
    SparkFlexConfig motorConfigL;
    SparkFlexConfig motorConfigR;

    //Create Memory PID Objects
    PIDController rPID;
    PIDController lPID;

    private double kP;
    private double kI;
    private double kD;

    // Target & Current Position
    // double targetHeight;
    // double currentHeight;

    double targetHeight;
    double currentHeight;

    double RtargetHeight;
    double RcurrentHeight;

    // Touch Sensor Objects
    DigitalInput touchSensorTop;
    DigitalInput touchSensorBottom;

    Pref<Double> kPPref = SKPreferences.attach("elevatorKp", 0.007)
        .onChange((newValue) -> {
            motorConfigL.closedLoop.p(newValue);
        });

    Pref<Double> kIPref = SKPreferences.attach("elevatorKi", 0.0)
        .onChange((newValue) -> {
            motorConfigL.closedLoop.i(newValue);
        });
    Pref<Double> kDPref = SKPreferences.attach("elevatorkD", 0.0)
        .onChange((newValue) -> {
            motorConfigL.closedLoop.d(newValue);
        });
    Pref<Double> kFFPref = SKPreferences.attach("elevatorkFF", 0.00196078431) // 1/500
        .onChange((newValue) -> {
            motorConfigL.closedLoop.velocityFF(newValue);
        });

    // @Override
    // public void initSendable(NTSendableBuilder builder) {
    //     SmartDashboard.putData(
    //         "ElevatorSubsystem",
    //         new Sendable() {
    //             @Override
    //             public void initSendable(SendableBuilder builder) {
    //                 builder.setSmartDashboardType("Elevator");

    //                 builder.addDoubleProperty(getName() + " kP",
    //                 () -> getKP(),
    //                 null);
    //                 builder.addDoubleProperty(getName() + " kI",
    //                 () -> getKI(),
    //                 () -> setKI());
    //                 builder.addDoubleProperty(getName() + " kD",
    //                 () -> getKD(),
    //                 () -> setKD());
    //             }
    //         }
    //     );
    // }

    // Constructor For Public Command Access
    public SK25Elevator()
    {
        // PID Controllers - Setpoints
        /*
        lPID = new PIDController(leftElevator.kP, leftElevator.kI, leftElevator.kD);
        rPID = new PIDController(rightElevator.kP, rightElevator.kI, rightElevator.kD);
        
        lPID.setIntegratorRange(kMinInteg, kMaxInteg);
        rPID.setIntegratorRange(kMinInteg, kMaxInteg);

        lPID.setSetpoint(0.0);
        rPID.setSetpoint(0.0);
        */

        // kP = 0.007;
        // kI = 0; //0.00075
        // kD = 0; //0.0001

        // Motor Initialization With REV Sparkflex - Configurations
        motorL = new SparkFlex(kLeftElevatorMotor.ID, MotorType.kBrushless);
        motorR = new SparkFlex(kRightElevatorMotor.ID, MotorType.kBrushless);

        motorConfigL = new SparkFlexConfig();
        motorConfigR = new SparkFlexConfig();

        // Encoder V3
        closedLoopController = motorL.getClosedLoopController();
        encoder = motorL.getEncoder();

        // Configurations For The Left Motor & Encoder
        motorConfigL
            .idleMode(IdleMode.kCoast).smartCurrentLimit(50).voltageCompensation(12);
        motorConfigL.encoder
            .positionConversionFactor(1)
            .velocityConversionFactor(1);
        motorConfigL.limitSwitch
            .reverseLimitSwitchEnabled(true)
            .reverseLimitSwitchType(Type.kNormallyOpen);
        motorConfigL.closedLoop
            .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
            //Set PID values for position control. We don't need to pass a closed loop
            //slot, as it will default to slot 0.
            .velocityFF(kFFPref.get())
            .p(kPPref.get())
            .i(kIPref.get()) 
            .d(kDPref.get()) 
            .outputRange(-1, 1)
            .maxMotion
            .maxAcceleration(6000)
            .maxVelocity(4200)
            .allowedClosedLoopError(.5);
        
        // Apply Motor Configurations
        motorL.configure(motorConfigL, SparkBase.ResetMode.kResetSafeParameters, SparkBase.PersistMode.kNoPersistParameters);

        // Configurations For The Right Motor
        motorConfigR
            .follow(motorL, true)
            .idleMode(IdleMode.kBrake);
            //.smartCurrentLimit(kElevatorCurrentLimit);
            //.inverted(true)

        motorR.configure(motorConfigR, SparkBase.ResetMode.kResetSafeParameters, SparkBase.PersistMode.kNoPersistParameters);

        // Current, Target, and Reset Positions
        // RtargetHeight = 0.0;
        // RcurrentHeight = 0.0;
        
        targetHeight = 0.0;
        currentHeight = 0.0;

        closedLoopController.setReference(0, ControlType.kPosition, ClosedLoopSlot.kSlot0);
    }

    public void stop() {
        closedLoopController.setReference(0, ControlType.kVelocity, ClosedLoopSlot.kSlot0);
    }
    
    /**
     * {@inheritDoc}
     */
    /*
    public void setRightTargetHeight(double height)
    {
        RtargetHeight = height;
        rPID.setSetpoint(LtargetHeight);
    }
    */
    
    /**
     * {@inheritDoc}
     */
    public void setTargetHeight(ElevatorPosition pos)
    {
        setTargetHeight(pos.height);
    }

    public void setTargetHeight(double targetHeight) 
    {
        this.targetHeight = targetHeight;
        closedLoopController.setReference(this.targetHeight, ControlType.kMAXMotionPositionControl);
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
    public double getTargetPosition()
    {
        return targetHeight;
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
    public boolean isAtTargetPosition()
    {
        return Math.abs(getEncoderPosition() - getTargetPosition()) < kPositionTolerance;
    }

    // Button Sensor Methods
    /*
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
    */

    @Override
    public void periodic()
    {  
        // Display encoder position and velocity
        SmartDashboard.putNumber("Actual Position", encoder.getPosition());
        SmartDashboard.putNumber("Actual Velocity", encoder.getVelocity());

        // Initialize Current & Target Positions
        double currentPosition = getEncoderPosition();
        // double lTargetPosition = getLeftTargetPosition();
        double targetPosition = getTargetPosition();
        // double rTargetPosition = getRightTargetPosition();
        
        // Calculates Motor Speed & Puts It Within Operating Range
        /* 
        double lSpeed = MathUtil.clamp(lPID.calculate(currentPosition), kElevatorMotorMinOutput, kElevatorMotorMaxOutput);
        double rSpeed = MathUtil.clamp(rPID.calculate(currentPosition), kElevatorMotorMinOutput, kElevatorMotorMaxOutput);

        motorL.set(lSpeed); 
        motorR.set(rSpeed); 
        */
        
        // SmartDashboard Current & Target Positions
        SmartDashboard.putNumber("Current Estimated Position", currentPosition);
        SmartDashboard.putNumber("Target Position", targetPosition);
        SmartDashboard.putBoolean("Elevator at Setpoint", isAtTargetPosition());
        // SmartDashboard.putNumber("Right Target Position", rTargetPosition);
        // SmartDashboard.putBoolean("Right Elevator at Setpoint", isRightAtTargetPosition()); 
    }
    public void testPeriodic(){}
    public void testInit(){}
}