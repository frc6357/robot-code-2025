package frc.robot.utils.elevatorPosition;

import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.RelativeEncoder;
import edu.wpi.first.wpilibj.DigitalInput;

import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.config.ClosedLoopConfig;

//import com.revrobotics.SparkFlexPIDController;

/**
 * Specific class to set the height of an elevator using a CAN Spark Flex Brushless motor with an
 * internal encoder, and to determine when it is at it's max point and zero point using
 * digital input sensors.
 */

public class SparkFlexElevator extends GenericElevatorMotor
{
    double                    Kp;
    double                    Ki;
    double                    Kd;
    double                    Kiz;
    boolean                   isLowerPresent;
    boolean                   isUpperPresent;
    double                    positionSetPoint;
    double                    heightSetPoint;
    SparkFlex                 motor;
    SparkFlexConfig           config;
    SparkClosedLoopController config2;
    RelativeEncoder           encoder;
    ClosedLoopConfig          pidController;
    double                    gearRatio;
    DigitalInput              UpperSensor;
    DigitalInput              LowerSensor;

    /**
     * Creates a new CAN Spark Max elevator
     * 
     * @param CanID
     *            Can ID of the motor used
     * @param gearRatio
     *            Number of motor shaft rotations per output shaft rotations
     * @param Kp
     *            Value for proportional gain constant in PID controller
     * @param Ki
     *            Value for integral gain constant in PID controller
     * @param Kd
     *            Value for derivative gain constant in PID controller
     * @param Kiz
     *            Value for I Zone constant in PID controller
     * @param LowerSensorID
     *            ID for digital input sensor that determines reset point of elevator
     * @param UpperSensorID
     *            ID for digital input sensor that determines max limit point of elevator or -1
     *            to indicate no switch is present
     */

    public SparkFlexElevator(int CanID, double gearRatio, double Kp, double Ki, double Kd, double Kiz, double MinOutput, double MaxOutput,
        int LowerSensorID, int UpperSensorID)
    {
        this(CanID, gearRatio, Kp, Ki, Kd, Kiz, MinOutput, MaxOutput, LowerSensorID);
        this.gearRatio = gearRatio;

        if (UpperSensorID != -1)
        {
            this.UpperSensor = new DigitalInput(UpperSensorID);
            isUpperPresent = true;
        }
        else
        {
            this.UpperSensor = null;
            isUpperPresent = false;
        }

    }

    /**
     * Creates a new CAN Spark Flex elevator
     * 
     * @param CanID
     *            Can ID of the motor used
     * @param gearRatio
     *            Number of motor shaft rotations per output shaft rotations
     * @param Kp
     *            Value for proportional gain constant in PID controller
     * @param Ki
     *            Value for integral gain constant in PID controller
     * @param Kd
     *            Value for derivative gain constant in PID controller
     * @param Kiz
     *            Value for I Zone constant in PID controller
     * @param LowerSensorID
     *            ID for digital input sensor that determines reset point of elevator
     */

    public SparkFlexElevator(int CanID, double gearRatio, double Kp, double Ki, double Kd, double Kiz, double MinOutput, double MaxOutput,
        int LowerSensorID)
    {
        this(CanID, gearRatio, Kp, Ki, Kd, Kiz, MinOutput, MaxOutput);

        if (LowerSensorID != -1)
        {
            this.LowerSensor = new DigitalInput(LowerSensorID);
            isLowerPresent = true;
        }
        else
        {
            this.LowerSensor = null;
            isLowerPresent = false;
        }
    }

    /**
     * Creates a new CAN Spark Flex elevator
     * 
     * @param CanID
     *            Can ID of the motor used
     * @param gearRatio
     *            Number of motor shaft rotations per output shaft rotations
     * @param Kp
     *            Value for proportional gain constant in PID controller
     * @param Ki
     *            Value for integral gain constant in PID controller
     * @param Kd
     *            Value for derivative gain constant in PID controller
     * @param Kiz
     *            Value for I Zone constant in PID controller
     */

    public SparkFlexElevator(int CanID, double gearRatio, double Kp, double Ki, double Kd, double Kiz, double MinOutput, double MaxOutput)
    {
        this.gearRatio = gearRatio;
        encoder = motor.getEncoder();
        config
            .idleMode(IdleMode.kBrake);
        config.encoder
            .positionConversionFactor(1.0);
        config.closedLoop
            //.feedbackSensor(FeedbackSensor.kPrimaryEncoder)
            .pid(Kp, Ki, Kd)
            .iZone(Kiz)
            .outputRange(MinOutput, MaxOutput);
            
        motor.configure(config, SparkBase.ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        isLowerPresent = false;
        isUpperPresent = false; 
    }

    public double getAppliedOutput()
    {
        return motor.getAppliedOutput();
    }

    public double getOutputCurrent()
    {
        return motor.getOutputCurrent();
    }

    public void resetEncoder()
    {
        encoder.setPosition(0.0); // Reset Position of encoder is 0.0
    }

    public void resetEncoder(double height)
    {
        encoder.setPosition(height * gearRatio / 360); // Reset position to native unit (rotations)

    }

    public void addFollowerMotor(int CanID)
    {
        try (SparkFlex followerMotor = new SparkFlex(CanID, MotorType.kBrushless))
        {
            config
                .follow(motor);
        }
    }

    public boolean isLowerAvailable()
    {
        return isLowerPresent;
    }

    public boolean isLowerReached()
    {
        return LowerSensor.get();
    }

    public boolean isUpperAvailable()
    {
        return isUpperPresent;
    }

    public boolean isUpperReached()
    {
        return UpperSensor.get();
    }

    public void stop()
    {
        motor.stopMotor();
    }
    public double getRightPosition()
    {
        double current_value = (encoder.getPosition() * 360) / gearRatio; // Convert native encoder unit of rotations to inches
        return current_value;
    }

    public double getLeftPosition()
    {
        double current_value = (encoder.getPosition() * 360) / gearRatio; // Convert native encoder unit of rotations to inches
        return current_value;
    }

    public double getRightTargetPosition()
    {
        return heightSetPoint;
    }
    public double getLeftTargetPosition()
    {
        return heightSetPoint;
    }

    public void setTargetPosition(double height)
    {
        heightSetPoint = height;
        positionSetPoint = (height * gearRatio) / 360.0;
        
        //TODO Hardcoded to solve the error, doesn't do what it's supposed to.
        config2
            .setReference(positionSetPoint, SparkFlex.ControlType.kPosition);
    }

    public void testInit()
    {
        config
            .idleMode(IdleMode.kCoast);
    }

    public void periodic()
    {
        
    }
}
