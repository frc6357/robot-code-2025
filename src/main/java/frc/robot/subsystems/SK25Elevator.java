// Essentials
package frc.robot.subsystems;
import frc.robot.subsystems.superclasses.Elevator;

// Constants (Muy Importante)
import frc.robot.Konstants.ElevatorConstants.ElevatorPosition;
import static frc.robot.Konstants.ElevatorConstants.kPositionTolerance;

// Ports
import static frc.robot.Ports.ElevatorPorts.kLeftElevatorMotor;
import static frc.robot.Ports.ElevatorPorts.kRightElevatorMotor;

// Relative Encoder
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
import com.revrobotics.spark.config.MAXMotionConfig.MAXMotionPositionMode;
import com.revrobotics.spark.SparkLimitSwitch;

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

    // Limit Switch
    SparkLimitSwitch forwardLimitSwitch;
    SparkLimitSwitch reverseLimitSwitch;

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
    Pref<Double> kFFPref = SKPreferences.attach("elevatorkFF", 0.00333333333) // 1/300 // 1/500
        .onChange((newValue) -> {
            motorConfigL.closedLoop.velocityFF(newValue);
        });

    // Constructor For Public Command Access
    public SK25Elevator()
    {
        // Motor initialization with REV
        motorL = new SparkFlex(kLeftElevatorMotor.ID, MotorType.kBrushless);
        motorR = new SparkFlex(kRightElevatorMotor.ID, MotorType.kBrushless);

        motorConfigL = new SparkFlexConfig();
        motorConfigR = new SparkFlexConfig();

        // Closed loop control with REV
        closedLoopController = motorL.getClosedLoopController();

        // Encoder with REV through bore (external encoder attached to the shaft)
        encoder = motorL.getEncoder();

        // Limit switches with REV
        forwardLimitSwitch = motorL.getForwardLimitSwitch();
        reverseLimitSwitch = motorL.getReverseLimitSwitch();

        // Configurations for the left motor, limit switch, encoder, & closed loop control
        motorConfigL
            .idleMode(IdleMode.kCoast).smartCurrentLimit(50).voltageCompensation(12);
        motorConfigL.encoder
            .positionConversionFactor(1)
            .velocityConversionFactor(1);
        motorConfigL.limitSwitch
            .forwardLimitSwitchType(Type.kNormallyOpen)
            .forwardLimitSwitchEnabled(true)
            .reverseLimitSwitchType(Type.kNormallyOpen)
            .reverseLimitSwitchEnabled(true);
        motorConfigL.closedLoop
            .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
            .velocityFF(kFFPref.get())
            .p(kPPref.get())
            .i(kIPref.get()) 
            .d(kDPref.get()) 
            .outputRange(-1, 1)
            .maxMotion
            .maxAcceleration(6000)
            .maxVelocity(4200)
            .allowedClosedLoopError(.5)
            .positionMode(MAXMotionPositionMode.kMAXMotionTrapezoidal);
        
        // Apply motor configurations on the left motor
        motorL.configure(motorConfigL, SparkBase.ResetMode.kResetSafeParameters, SparkBase.PersistMode.kNoPersistParameters);

        // Configurations for the right motor (follower)
        motorConfigR
            .follow(motorL, true)
            .idleMode(IdleMode.kBrake);

        // Apply motor configurations on the right motor
        motorR.configure(motorConfigR, SparkBase.ResetMode.kResetSafeParameters, SparkBase.PersistMode.kNoPersistParameters);

        // Initialize current, target, and reset positions
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

    public void setPositionFromAbove(ElevatorPosition pos) {

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
        // Current & Target Positions
        double currentPosition = getEncoderPosition();
        double targetPosition = getTargetPosition();
        
        // SmartDashboard Current & Target Positions
        SmartDashboard.putNumber("Current Estimated Position", currentPosition);
        SmartDashboard.putNumber("Target Position", targetPosition);
        SmartDashboard.putBoolean("Elevator at Setpoint", isAtTargetPosition());

        // Display encoder position and velocity
        SmartDashboard.putNumber("Actual Position", encoder.getPosition());
        SmartDashboard.putNumber("Actual Velocity", encoder.getVelocity());

        // Display if limit switch is "pressed" (in our case, passed)
        SmartDashboard.putBoolean("Forward Limit Reached", forwardLimitSwitch.isPressed());
        SmartDashboard.putBoolean("Reverse Limit Reached", reverseLimitSwitch.isPressed());
    }
    public void testPeriodic(){}
    public void testInit(){}
}