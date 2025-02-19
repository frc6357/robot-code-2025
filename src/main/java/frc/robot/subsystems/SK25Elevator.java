// Essentials
package frc.robot.subsystems;
import frc.robot.subsystems.superclasses.Elevator;
//import edu.wpi.first.math.MathUtil;

// Constants (Muy Importante)
import frc.robot.Konstants.ElevatorConstants.ElevatorPosition;
import static frc.robot.Konstants.ElevatorConstants.kPositionTolerance;

// Ports
import static frc.robot.Ports.ElevatorPorts.kLeftElevatorMotor;
import static frc.robot.Ports.ElevatorPorts.kRightElevatorMotor;

// Encoder V3
import com.revrobotics.RelativeEncoder;

// Closed Loop
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

// Motors - Sparkflex
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;

// Configurations For Motors
import com.revrobotics.spark.config.SparkFlexConfig;

// Limit Switches
import com.revrobotics.spark.config.LimitSwitchConfig.Type;

// SmartDashboard
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

// Preferences
import frc.robot.preferences.Pref;
import frc.robot.preferences.SKPreferences;

public class SK25Elevator extends Elevator
{
    // Create Memory Motor Objects
    SparkFlex motorR;
    SparkFlex motorL;

    // Encoder & ClosedLoop
    SparkClosedLoopController closedLoopController;
    RelativeEncoder encoder;

    // Creating Config Object
    SparkFlexConfig motorConfigL;
    SparkFlexConfig motorConfigR;

    // Target & Current Position
    double targetHeight;
    double currentHeight;

    // SKPreferences for PID
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

    // Constructor For Public Command Access
    public SK25Elevator()
    {
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

        // Apply Motor Configurations
        motorR.configure(motorConfigR, SparkBase.ResetMode.kResetSafeParameters, SparkBase.PersistMode.kNoPersistParameters);

        // Current, Target, and Reset Positions
        targetHeight = 0.0;
        currentHeight = 0.0;
        closedLoopController.setReference(0, ControlType.kPosition, ClosedLoopSlot.kSlot0);
    }
    
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
    public double getTargetPosition()
    {
        return targetHeight;
    }

    /**
     * {@inheritDoc}
     */
    public boolean isAtTargetPosition()
    {
        return Math.abs(getEncoderPosition() - getTargetPosition()) < kPositionTolerance;
    }

    @Override
    public void periodic()
    {  
        // Display encoder position and velocity
        SmartDashboard.putNumber("Actual Position", encoder.getPosition());
        SmartDashboard.putNumber("Actual Velocity", encoder.getVelocity());

        // Initialize Current & Target Positions
        double currentPosition = getEncoderPosition();
        double targetPosition = getTargetPosition();
        
        // SmartDashboard Current & Target Positions
        SmartDashboard.putNumber("Current Estimated Position", currentPosition);
        SmartDashboard.putNumber("Target Position", targetPosition);
        SmartDashboard.putBoolean("Elevator at Setpoint", isAtTargetPosition());
    }
    public void testPeriodic(){}
    public void testInit(){}
}
