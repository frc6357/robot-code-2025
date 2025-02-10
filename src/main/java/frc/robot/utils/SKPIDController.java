package frc.robot.utils;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.trajectory.TrapezoidProfile;

// ProfiledPIDController class description found here:
// https://github.wpilib.org/allwpilib/docs/release/java/edu/wpi/first/math/controller/ProfiledPIDController.html
//TrapezoidProfile class description found here:
// https://github.wpilib.org/allwpilib/docs/release/java/edu/wpi/first/math/trajectory/TrapezoidProfile.html
// Feedforward class description found here:
// https://github.wpilib.org/allwpilib/docs/release/java/edu/wpi/first/math/controller/SimpleMotorFeedforward.html


/** This class is intended to provide additional functionality for the PIDController class
 * as well as making it easier and more standardized to use.*/
public class SKPIDController extends ProfiledPIDController{
    
    private double P;
    private double I;
    private double D;
    private TrapezoidProfile profile;
    private TrapezoidProfile.State currentProfileState;
    private TrapezoidProfile.State targetProfileState;
    private SimpleMotorFeedforward feedForward;

    public SKPIDController(double kP, double kI, double kD, TrapezoidProfile.Constraints constraints)
    {
        super(kP, kI, kD, constraints);
        this.P = kP;
        this.I = kI;
        this.D = kD;
        this.profile = new TrapezoidProfile(constraints);
        //if no target provided, assume the current position is the target
        targetProfileState = currentProfileState;
        feedForward = new SimpleMotorFeedforward(0, 0);
    }

    //auto constructor
    public SKPIDController(
        double kP, 
        double kI, 
        double kD, 
        TrapezoidProfile.Constraints constraints, 
        double voltGains, 
        double velocityGains, 
        double acceleratoinGains,
        TrapezoidProfile.State currentProfileState,
        TrapezoidProfile.State targetProfileState)
    {
        super(kP, kI, kD, constraints);
        this.P = kP;
        this.I = kI;
        this.D = kD;
        this.profile = new TrapezoidProfile(constraints);
        this.currentProfileState = currentProfileState;
        this.targetProfileState = targetProfileState;
        feedForward = new SimpleMotorFeedforward(voltGains, velocityGains, acceleratoinGains);
    }


    // velocity in m/s
    public double getConstantFeedForward(double velocity)
    {
        return feedForward.calculate(velocity);
    }

    // velocity in m/s and acclereration in m/s^2
    public double getAcceleratingFeedForward(double velocity, double acceleration)
    {
        return feedForward.calculate(velocity, acceleration);
    }

    //replace the old feedforward values with new ones, dosnt updated calculations preformed before 
    //this action. kS is volts gain kV is velocity gain, and kA is acceleration gain. A low volts 
    //value is recommended.
    public void setNewFeedForward(double kS, double kV, double kA)
    {
        feedForward = new SimpleMotorFeedforward(kS, kV, kA);
    }

    public SimpleMotorFeedforward getSimpleMotorFeedforward()
    {
        return feedForward;
    }

    //add feedfoward (one for raw values, one for motor feedforward type)
    //current and target pos can be either meters or radians
    public double getConstantOutputWithFeedForward(double currentPoint, double targetPoint, double velocity)
    {
        return (this.calculate(currentPoint, targetPoint) + getConstantFeedForward(velocity));
    }

    //add feedfoward (one for raw values, one for motor feedforward type)
    //current and target pos can be either meters or radians
    public double getConstantOutputWithFeedForward(double currentPoint, TrapezoidProfile.State targetProfile, double velocity)
    {
        return (this.calculate(currentPoint, targetProfile) + getConstantFeedForward(velocity));
    }

    //add feedfoward (one for raw values, one for motor feedforward type)
    //current and target pos can be either meters or radians
    public double getAcceleratingOutputWithFeedForward(double currentPoint, double targetPoint, double velocity, double acceleration)
    {
        return (this.calculate(currentPoint, targetPoint) + getAcceleratingFeedForward(velocity, acceleration));
    }

    /** Calculates the difference between the current position and target position.
     * This overriden method uses the targetprofileState from the constructor. If no target
     * profile state was included, assume the target state is the current state.
     * @param currentPos The current position of the motor (encoder reading) in meters or radians.
     * @return The distance to travel as the next PID controller output. The units will be meters or 
     * radians 
     */
    @Override
    public double calculate(double currentPos)
    {
        return this.calculate(currentPos, targetProfileState);
    }

    //auto PID method
}
