// Essentials
package frc.robot.subsystems;
import static frc.robot.Konstants.ElevatorConstants.kPositionTolerance;
// Ports
import static frc.robot.Ports.ElevatorPorts.kLeftElevatorMotor;
import static frc.robot.Ports.ElevatorPorts.kRightElevatorMotor;

import java.util.function.Supplier;

// Relative Encoder
import com.revrobotics.RelativeEncoder;
// Closed Loop
import com.revrobotics.spark.ClosedLoopSlot;
// Motors - Sparkflex
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLimitSwitch;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
// Limit Switches
import com.revrobotics.spark.config.LimitSwitchConfig.Type;
import com.revrobotics.spark.config.MAXMotionConfig.MAXMotionPositionMode;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
// Configurations For Motors
import com.revrobotics.spark.config.SparkFlexConfig;

// SmartDashboard
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
// Constants (Muy Importante)
import frc.robot.Konstants.ElevatorConstants.ElevatorPosition;
// Preferences
import frc.robot.preferences.Pref;
import frc.robot.preferences.SKPreferences;
import frc.robot.subsystems.superclasses.Elevator;
import frc.robot.utils.Util;

public class SK25Elevator extends Elevator
{
    // Create Memory Motor Objects
    SparkFlex motorR;
    //SparkFlex motorL;

    // Encoder & ClosedLoop
    SparkClosedLoopController closedLoopController;
    public RelativeEncoder encoder;

    // Creating Config Object
    SparkFlexConfig motorConfigL;
    SparkFlexConfig motorConfigR;

    // Target & Current Position
    double targetHeight;
    public double currentHeight;

    // Limit Switch
    //SparkLimitSwitch forwardLimitSwitch;
    //SparkLimitSwitch reverseLimitSwitch;

    double actualFF;

    // SKPreferences for PID & FF
    public Pref<Double> kPPref = SKPreferences.attach("elevatorKp", 0.5)
        .onChange((newValue) -> {
            motorConfigR.closedLoop.p(newValue);
            motorR.configure(motorConfigR, SparkBase.ResetMode.kResetSafeParameters, SparkBase.PersistMode.kNoPersistParameters);
        });
    public Pref<Double> kIPref = SKPreferences.attach("elevatorKi", 0.0)
        .onChange((newValue) -> {
            motorConfigR.closedLoop.i(newValue);
            motorR.configure(motorConfigR, SparkBase.ResetMode.kResetSafeParameters, SparkBase.PersistMode.kNoPersistParameters);
        });
    public Pref<Double> kDPref = SKPreferences.attach("elevatorkD", 0.01)
        .onChange((newValue) -> {
            motorConfigR.closedLoop.d(newValue);
            motorR.configure(motorConfigR, SparkBase.ResetMode.kResetSafeParameters, SparkBase.PersistMode.kNoPersistParameters);
        });
    public Pref<Double> kFFPref = SKPreferences.attach("elevatorkVFF", 0.0) // Formula's asymptote FF value
        .onChange((newValue) -> {
            motorConfigR.closedLoop.velocityFF(newValue);
            motorR.configure(motorConfigR, SparkBase.ResetMode.kResetSafeParameters, SparkBase.PersistMode.kNoPersistParameters);
        });
        public Pref<Double> kFFpPref = SKPreferences.attach("elevatorkpFF", 0.008) // Formula's asymptote FF value
        .onChange((newValue) -> {
            motorConfigR.closedLoop.velocityFF(newValue);
            motorR.configure(motorConfigR, SparkBase.ResetMode.kResetSafeParameters, SparkBase.PersistMode.kNoPersistParameters);
        });

    // Constructor For Public Command Access
    public SK25Elevator()
    {
        // Motor initialization with REV
        //motorL = new SparkFlex(kLeftElevatorMotor.ID, MotorType.kBrushless);
        motorR = new SparkFlex(kRightElevatorMotor.ID, MotorType.kBrushless);

        motorConfigL = new SparkFlexConfig();
        motorConfigR = new SparkFlexConfig();

        // Closed loop control with REV
        closedLoopController = motorR.getClosedLoopController();

        // Encoder with REV through bore (external encoder attached to the shaft)
        encoder = motorR.getExternalEncoder();

        // Limit switches with REV
        //forwardLimitSwitch = motorL.getForwardLimitSwitch();
        //reverseLimitSwitch = motorL.getReverseLimitSwitch();

        // Configurations for the left motor, limit switch, encoder, & closed loop control
        motorConfigR
            .inverted(true)
            .idleMode(IdleMode.kCoast);//.smartCurrentLimit(50);
        motorConfigR.softLimit
            .reverseSoftLimitEnabled(false);
        motorConfigR.externalEncoder
            .inverted(true)
            .positionConversionFactor(1)
            .velocityConversionFactor(0.01666667); // RPM -> RPS (Divide by 60)
        //motorConfigL.limitSwitch
            //.forwardLimitSwitchType(Type.kNormallyOpen)
            //.forwardLimitSwitchEnabled(true)
            //.reverseLimitSwitchType(Type.kNormallyOpen)
            //.reverseLimitSwitchEnabled(true);
        motorConfigR.closedLoop
            .feedbackSensor(FeedbackSensor.kAlternateOrExternalEncoder)
            .pidf(kPPref.get(), kIPref.get(),kDPref.get(), kFFpPref.get()) 
            .outputRange(-.3, .3);
            // .dFilter(0.3)
            // .iZone(0.1);
            //.velocityFF(kFFPref.get()) (Taken care of in pidf)
        motorConfigR.closedLoop.maxMotion
            .maxAcceleration(200)
            .maxVelocity(100)
            .allowedClosedLoopError(0.005);
            // .positionMode(MAXMotionPositionMode.kMAXMotionTrapezoidal);
        
        // Apply motor configurations on the left motor
        motorR.configure(motorConfigR, SparkBase.ResetMode.kNoResetSafeParameters, SparkBase.PersistMode.kPersistParameters);

        // Configurations for the right motor (follower)
        //motorConfigL
            //.follow(motorR, true)
            //.idleMode(IdleMode.kCoast);

        // Apply motor configurations on the right motor
        //motorL.configure(motorConfigL, SparkBase.ResetMode.kResetSafeParameters, SparkBase.PersistMode.kNoPersistParameters);

        // Initialize current, target, and reset positions
        targetHeight = 0.0;
        currentHeight = 0.0;
        encoder.setPosition(0);
        closedLoopController.setReference(0, ControlType.kMAXMotionPositionControl, ClosedLoopSlot.kSlot0);

        if(motorR.hasStickyFault() || motorR.hasStickyWarning())
        {
            motorR.clearFaults();
        }
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
        // closedLoopController.setReference(this.targetHeight, ControlType.kMAXMotionPositionControl, ClosedLoopSlot.kSlot0);
    }

    /**
     * {@inheritDoc}
     */
    public double calculateFF(double targetHeight) {
        if (targetHeight >= 11) {
            actualFF = 0.0025;
        }
        else if (targetHeight == 9.5){
            actualFF = 0.00254;
        }
        else if (targetHeight == 7){
            actualFF = 0.003;
        }
        else if (targetHeight == 3){
            actualFF = 0.0093;
        }
        else{
            double a = 0.06423;
            double b = 0.461;

            // Determined through graphing well-tuned FF values to find this formula 
            //TODO Tune the formula again
            double targetFF = (a * (Math.pow(b, targetHeight)) + 0.0025); // y = a(x^b)
            actualFF = Util.limit(targetFF, 0, 0.02); 
        }
        return actualFF;
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
        double vel = Math.abs(encoder.getVelocity()); // Revolutions / sec
        boolean atGoal = (Math.abs(getEncoderPosition() - getTargetPosition()) < kPositionTolerance);
        boolean lowVelocity = vel < 0.2;

        return atGoal && lowVelocity;
    }

    /**For use with SwerveBinder acceleration limits on the swerve based on the elevator height.
     * Uses a supplier to get the most recent value.
     * @return The current elevator height supplier.
     */
    public Supplier<Double> getCurrentHeightMotorRotations()
    {
        Supplier<Double> currentHeightSupplier = ()-> currentHeight;
        return currentHeightSupplier;
    }

    @Override
    public void periodic()
    {  
        // Current & Target Positions
        double currentPosition = getEncoderPosition();
        double targetPosition = getTargetPosition();

        closedLoopController.setReference(this.targetHeight, ControlType.kMAXMotionPositionControl);
        
        // SmartDashboard Current & Target Positions
        SmartDashboard.putNumber("Current Estimated Position", currentPosition);
        SmartDashboard.putNumber("Target Position", targetPosition);
        SmartDashboard.putBoolean("Elevator at Setpoint", isAtTargetPosition());
        SmartDashboard.putNumber("Actual FF", actualFF);

        // Display encoder position and velocity
        SmartDashboard.putNumber("Actual Position", encoder.getPosition());
        SmartDashboard.putNumber("Actual Velocity", encoder.getVelocity());

        // Display if limit switch is "pressed" (in our case, passed)
        //SmartDashboard.putBoolean("Forward Limit Reached", forwardLimitSwitch.isPressed());
        //SmartDashboard.putBoolean("Reverse Limit Reached", reverseLimitSwitch.isPressed());
    }
    public void testPeriodic(){}
    public void testInit(){}
}
