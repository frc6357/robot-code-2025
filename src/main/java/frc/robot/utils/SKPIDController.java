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


/** This wrapper class is intended to provide additional functionality for the PIDController class
 * as well as making it easier and more standardized to use. Note that either meters or radians can 
 * be used as units, but any values passed through the constructor and methods of one object must be 
 * the same unit type. It is also highly recommended to call one of the available reset() or 
 * setDistanceAndVelocity() methods before doin any calculations or applications with the PID 
 * controller.*/
public class SKPIDController extends ProfiledPIDController{
    
    private TrapezoidProfile profile;
    private TrapezoidProfile.Constraints constraints;
    private TrapezoidProfile.State currentProfileState;
    private TrapezoidProfile.State targetProfileState;
    private SimpleMotorFeedforward feedForward;

    /** Creates a new SKPIDController object, which uses methods and logic from the super class
     * ProfiledPIDController to provide convenience methods and descriptions which make actions 
     * easier to understand and preform, often requireing less arguments. Also adds new methods to
     * implement feedforward, and methods to apply any values or conditions for PID to a motor in one 
     * method. This controller uses a trapezoid profile, meaning that it accelerates to a certain speed 
     * before remaining at that speed until it nears its target, then decreases in speed to arrive at the 
     * target. The velocity-time graph of these motions ressembles a trapezoid, hence the name. Note that 
     * either meters or radians can be used as units, but any values passed through the constructor and 
     * methods of one object must be the same unit type. It is also highly recommended to call one of the
     * available reset() or setDistanceAndVelocity() methods before doin any calculations or applications
     * with the PID controller.
     * @param kP The Porportional (P) constant of the PID controller. This adjusts the PID 
     * controller's target, and should not go below zero.
     * @param kP The Integral (I) constant of the PID controller. This adjusts the PID 
     * controller's speed to reach the target, and should not go below zero. this value is almost never 
     * used in frc, and it is not recommended to use a value aobve zero.
     * @param kP The Derivative (D) constant of the PID controller. This adjusts the PID 
     * controller's ability to slow down when it nears its target, and should not go below zero.
     * @param constraints The constraints trapezoid profile object which sets the acceleration for the
     * trapezoid profile (specified in the TrapezoidProfile.Constraints object) to accelerate toward until 
     * it reaches its max velocity property (also specified in the TrapezoidProfile.Constraints object.)
      */
    public SKPIDController(
        double kP, 
        double kI, 
        double kD, 
        TrapezoidProfile.Constraints constraints
        )
    {
        super(kP, kI, kD, constraints);
        this.constraints = constraints;
        this.profile = new TrapezoidProfile(constraints);
        //set the current state to position of zero and velocity of zero.
        currentProfileState = new TrapezoidProfile.State();
        //if no target is provided, assume the current position is the target
        targetProfileState = currentProfileState;
        //feedforward values are zero if none are provided
        feedForward = new SimpleMotorFeedforward(0, 0, 0);
        //reset the PID controller with the position and velocity from the current state so those 
        //respective defualt values exist before using the PID controller.
        setPositionAndVelocity(currentProfileState);
    }

    /** Creates a new SKPIDController object, which uses methods and logic from the super class
     * ProfiledPIDController to provide convenience methods and descriptions which make actions 
     * easier to understand and preform, often requireing less arguments. Also adds new methods to
     * implement feedforward, and methods to apply any values or conditions for PID to a motor in one 
     * method. This controller uses a trapezoid profile, meaning that it accelerates to a certain speed 
     * before remaining at that speed until it nears its target, then decreases in speed to arrive at the 
     * target. The velocity-time graph of these motions ressembles a trapezoid, hence the name. Note that 
     * either meters or radians can be used as units, but any values passed through the constructor and 
     * methods of one object must be the same unit type. It is also highly recommended to call one of the
     * available reset() or setDistanceAndVelocity() methods before doin any calculations or applications
     * with the PID controller.
     * @param kP The Porportional (P) constant of the PID controller. This adjusts the PID 
     * controller's target, and should not go below zero.
     * @param kP The Integral (I) constant of the PID controller. This adjusts the PID 
     * controller's speed to reach the target, and should not go below zero. this value is almost never 
     * used in frc, and it is not recommended to use a value aobve zero.
     * @param kP The Derivative (D) constant of the PID controller. This adjusts the PID 
     * controller's ability to slow down when it nears its target, and should not go below zero.
     * @param constraints The constraints trapezoid profile object which sets the acceleration for the
     * trapezoid profile (specified in the TrapezoidProfile.Constraints object) to accelerate toward until 
     * it reaches its max velocity property (also specified in the TrapezoidProfile.Constraints object.)
     * @param voltGains The static volts gain feedforward value to apply to the controller to reduce voltage 
     * output error.
     * @param velocityGains The velocity gain feedforward value to apply to the controller to reduce velocity
     * output error.
     * @param accelerationGains The acceleration gain feedforward value to apply to the controller to reduce acceleration
     * output error.
     * @param currentProfileState The object containing the current state of the PID controller. This
     * includes the current position (meters or radians) and the current velocity (m/s or rad/s) of the 
     * controller.
     * @param targetProfileState The object containing the target state of the PID controller. This
     * includes the target position (meters or radians) and the target velocity (m/s or rad/s) of the 
     * controller.
      */
    public SKPIDController(
        double kP, 
        double kI, 
        double kD, 
        TrapezoidProfile.Constraints constraints, 
        double voltGains, 
        double velocityGains, 
        double accelerationGains,
        TrapezoidProfile.State currentProfileState,
        TrapezoidProfile.State targetProfileState
        )
    {
        super(kP, kI, kD, constraints);
        this.constraints = constraints;
        this.profile = new TrapezoidProfile(constraints);
        this.currentProfileState = currentProfileState;
        this.targetProfileState = targetProfileState;
        feedForward = new SimpleMotorFeedforward(voltGains, velocityGains, accelerationGains);
        //reset the PID controller with the position and velocity from the current state so those 
        //respective defualt values exist before using the PID controller.
        setPositionAndVelocity(currentProfileState);
    }

     /** Creates a new SKPIDController object, which uses methods and logic from the super class
     * ProfiledPIDController to provide convenience methods and descriptions which make actions 
     * easier to understand and preform, often requireing less arguments. Also adds new methods to
     * implement feedforward, and methods to apply any values or conditions for PID to a motor in one 
     * method. This controller uses a trapezoid profile, meaning that it accelerates to a certain speed 
     * before remaining at that speed until it nears its target, then decreases in speed to arrive at the 
     * target. The velocity-time graph of these motions ressembles a trapezoid, hence the name. Note that 
     * either meters or radians can be used as units, but any values passed through the constructor and 
     * methods of one object must be the same unit type. It is also highly recommended to call one of the
     * available reset() or setDistanceAndVelocity() methods before doin any calculations or applications
     * with the PID controller.
     * @param kP The Porportional (P) constant of the PID controller. This adjusts the PID 
     * controller's target, and should not go below zero.
     * @param kP The Integral (I) constant of the PID controller. This adjusts the PID 
     * controller's speed to reach the target, and should not go below zero. this value is almost never 
     * used in frc, and it is not recommended to use a value aobve zero.
     * @param kP The Derivative (D) constant of the PID controller. This adjusts the PID 
     * controller's ability to slow down when it nears its target, and should not go below zero.
     * @param constraints The constraints trapezoid profile object which sets the acceleration for the
     * trapezoid profile (specified in the TrapezoidProfile.Constraints object) to accelerate toward until 
     * it reaches its max velocity property (also specified in the TrapezoidProfile.Constraints object.)
     * @param voltGains The static volts gain feedforward value to apply to the controller to reduce voltage 
     * output error.
     * @param velocityGains The velocity gain feedforward value to apply to the controller to reduce velocity
     * output error.
     * @param accelerationGains The acceleration gain feedforward value to apply to the controller to reduce acceleration
     * output error.
     * @param currentPosition The current position of the controller, usually fetched by the encoder and
     * converted to either meters or radians.
     * @param currentVelocity The current velocity of the controller in m/s or rad/s (usually fetched 
     * from encoder.)
     * @param targetPosition the target position of the controller in either meters or radians.
     * @param targetVelocity The target velocity for the controller to reach in m/s or rad/s. If the controller
     * should speed up, the max speed used for the TrapezoidProfile.Constraints object can be used to increase
     * the target output speed, or another target velocity can be used. If the controller should slow to 
     * a stop, this value can be zero.
      */
    public SKPIDController(
        double kP, 
        double kI, 
        double kD, 
        TrapezoidProfile.Constraints constraints, 
        double voltGains, 
        double velocityGains, 
        double acceleratoinGains,
        double currentPosition,
        double currentVelocity,
        double targetPosition,
        double targetVelocity
        )
    {
        super(kP, kI, kD, constraints);
        this.constraints = constraints;
        this.profile = new TrapezoidProfile(constraints);
        this.currentProfileState = new TrapezoidProfile.State(currentPosition, currentVelocity);
        this.targetProfileState = new TrapezoidProfile.State(targetPosition, targetVelocity);
        feedForward = new SimpleMotorFeedforward(voltGains, velocityGains, acceleratoinGains);
        //reset the PID controller with the position and velocity from the current state so those 
        //respective defualt values exist before using the PID controller.
        setPositionAndVelocity(currentProfileState);
    }




    //Methods to set the PID and feedforward constants.




    /** Changes the existing P, I, and D constants to the new values. This change will not affect 
     * any logic enacted before calling this method. 
     * @param newKP The new Porportional (P) constant of the PID controller. This adjusts the PID 
     * controller's target, and should not go below zero.
     * @param newKI The new Integral (I) constant of the PID controller. This adjusts the PID 
     * controller's speed to reach the target, and should not go below zero. this value is almost never 
     * used in frc, and it is not recommended to use a value aobve zero.
     * @param newKD The new Derivative (D) constant of the PID controller. This adjusts the PID 
     * controller's ability to slow down when it nears its target, and should not go below zero.
     * @param constraints The constraints trapezoid profile object which sets the acceleration for the
     * trapezoid profile (specified in the TrapezoidProfile.Constraints object) to accelerate toward until 
     * it reaches its max velocity property (also specified in the TrapezoidProfile.Constraints object.)
     * */
    public void setPIDConstants(double newKP, double newKI, double newKD)
    {
        this.setP(newKP);    
        this.setI(newKI);
        this.setD(newKD);
    }

    /** Replaces the old feedforward values with new ones, dosnt updated calculations preformed before 
     * this action. Applies a SimpleMotorFeedforward object to the PID controller, constructed using the 
     * arguments. Feedforward applies constant values to reduce the error present in the controller output by 
     * adjusting the output values by a factor of the feeforward values. The acceleration gain constant
     * (kA) is zero.
     * @param kS The new volt gain feedforward constant which determines the amount of volts to apply to an 
     * error. A low value is recommended.
     * @param kV The new velocity gain constant which determines how much velocity to add to the velocity
     * error.
     */
    public void setNewFeedForward(double kS, double kV)
    {
        feedForward = new SimpleMotorFeedforward(kS, kV, 0);
    }

    /** Replaces the old feedforward values with new ones, dosnt updated calculations preformed before 
     * this action. Applies a SimpleMotorFeedforward object to the PID controller, constructed using the 
     * arguments. Feedforward applies constant values to reduce the error present in the controller output by 
     * adjusting the output values by a factor of the feeforward values. 
     * @param kS The new volt gain feedforward constant which determines the amount of volts to apply to an 
     * error. A low value is recommended.
     * @param kV The new velocity gain constant which determines how much velocity to add to the velocity
     * error.
     * @param kA The new acceleration gain constant which determines how much acceleration to add to the 
     * acceleration error of the controller.
     */
    public void setNewFeedForward(double kS, double kV, double kA)
    {
        feedForward = new SimpleMotorFeedforward(kS, kV, kA);
    }

    /** Changes the existing P, I, and D constants to the new values in addition to the feedforward constants
     * kS, kV, and kA. This change will not affect any logic enacted before calling this method. 
     * Applies a SimpleMotorFeedforward object to the PID controller, constructed using the 
     * arguments. Feedforward applies constant values to reduce the error present in the controller output by 
     * adjusting the output values by a factor of the feeforward values. The acceleration gain is zero.
     * @param newKP The new Porportional (P) constant of the PID controller. This adjusts the PID 
     * controller's target, and should not go below zero.
     * @param newKI The new Integral (I) constant of the PID controller. This adjusts the PID 
     * controller's speed to reach the target, and should not go below zero. this value is almost never 
     * used in frc, and it is not recommended to use a value aobve zero.
     * @param newKD The new Derivative (D) constant of the PID controller. This adjusts the PID 
     * controller's ability to slow down when it nears its target, and should not go below zero.
     * @param kS The new volt gain feedforward constant which determines the amount of volts to apply to an 
     * error. A low value is recommended.
     * @param kV The new velocity gain constant which determines how much velocity to add to the velocity
     * error.
     * */
    public void setPIDConstantsAndFeedForward(double newKP, double newKI, double newKD, double kS, double kV)
    {
        this.setP(newKP);    
        this.setI(newKI);
        this.setD(newKD);
        feedForward = new SimpleMotorFeedforward(kS, kV);
    }

    /** Changes the existing P, I, and D constants to the new values in addition to the feedforward constants
     * kS, kV, and kA. This change will not affect any logic enacted before calling this method. 
     * Applies a SimpleMotorFeedforward object to the PID controller, constructed using the 
     * arguments. Feedforward applies constant values to reduce the error present in the controller output by 
     * adjusting the output values by a factor of the feeforward values.
     * @param newKP The new Porportional (P) constant of the PID controller. This adjusts the PID 
     * controller's target, and should not go below zero.
     * @param newKI The new Integral (I) constant of the PID controller. This adjusts the PID 
     * controller's speed to reach the target, and should not go below zero. this value is almost never 
     * used in frc, and it is not recommended to use a value aobve zero.
     * @param newKD The new Derivative (D) constant of the PID controller. This adjusts the PID 
     * controller's ability to slow down when it nears its target, and should not go below zero.
     * @param kS The new volt gain feedforward constant which determines the amount of volts to apply to an 
     * error. A low value is recommended.
     * @param kV The new velocity gain constant which determines how much velocity to add to the velocity
     * error.
     * @param kA The new acceleration gain constant which determines how much acceleration to add to the 
     * acceleration error of the controller.
     * */
    public void setPIDConstantsAndFeedForward(double newKP, double newKI, double newKD, double kS, double kV, double kA)
    {
        this.setP(newKP);    
        this.setI(newKI);
        this.setD(newKD);
        feedForward = new SimpleMotorFeedforward(kS, kV, kA);
    }




    //Getter methods for TrapezoidProfile objects and subclass objects.




    /** Gets the trapezoid profile object from the PID controller with the applied constraints from this
     * object's constructor.
     * @return The trapezoid profile object.
     */
    public TrapezoidProfile getTrapezoidProfile()
    {
        return profile;
    }

    /** Gets the constraints object containing the max acceleration (m/s^2 or rad/s^2) and velocity 
     * (m/s or rad/s) profile for the controller. Overriden to better explain the functino of this method.
     * @return The TrapezoidProfile.Constraints constraints object.
     */
    @Override
    public TrapezoidProfile.Constraints getConstraints()
    {
        return constraints;
    }

    /** Gets the current profile state object containing the current position velocity (m/s or rad/s) 
     * state of the profile of the controller.
     * @return The current TrapezoidProfile.State object.
     */
    public TrapezoidProfile.State getCurrentState()
    {
        return currentProfileState;
    }

    /** Gets the target profile state object containing the target position and velocity states of
     * the profileof the controller.
     * @return The target TrapezoidProfile.State object which contains the velocity (m/s or rad/s) and 
     * acceleration (m/s^2 or rad/s^2). Returns the current state if no target state was set. 
     * If not current state was set, returns a state containing a position and velocity of zero.
     */
    public TrapezoidProfile.State getTargetState()
    {
        return targetProfileState;
    }

    /** Gets the feedforward object any containing feedforward values for the cotroller.
     * @return The SimpleMotorFeedForward object.
    */
    public SimpleMotorFeedforward getSimpleMotorFeedforward()
    {
        return feedForward;
    }




     //PID output caclulation Methods




    /** Gets the feedforward value for the next output of the controller using the velocity gain.
     * @param velocity The target velocity of the output in m/s or rad/s. 
     * @return The calculated feedforward value.
    */
    public double getConstantFeedForward(double velocity)
    {
        return feedForward.calculate(velocity);
    }

    /** Gets the feedforward value for the next output of the controller using the velocity gain
     * and the acceleration gain.
     * @param velocity The target velocity of the output in m/s or rad/s. 
     * @param acceleration The target aceleration of the controller in m/s^2 or rad/s^2.
     * @return The calculated feedforward value.
    */
    public double getAcceleratingFeedForward(double velocity, double acceleration)
    {
        return feedForward.calculate(velocity, acceleration);
    }

    /** Gets the output of the PID controller after adding the feedforward output using the 
     * controller velocity.
     * @param currentPoint The current position of the controller in meters or radians.
     * @param targetPoint The target position of the controller in meters or radians.
     * @param velocity The target velocity of the controller in m/s or rad/s.
     * @return The PID controller output with applied feedforward.
    */
    public double getConstantOutputWithFeedForward(double currentPoint, double targetPoint, double velocity)
    {
        return (this.calculate(currentPoint, targetPoint) + getConstantFeedForward(velocity));
    }

    /** Gets the output of the PID controller after adding the feedforward output using the 
     * controller velocity.
     * @param currentPoint The current position of the controller in meters or radians.
     * @param targetProfile The target profile state object containing the target position and
     * velocity.
     * @param velocity The target velocity of the controller in m/s or rad/s.
     * @return The PID controller output with applied feedforward.
    */
    public double getConstantOutputWithFeedForward(double currentPoint, TrapezoidProfile.State targetProfile, double velocity)
    {
        return (this.calculate(currentPoint, targetProfile) + getConstantFeedForward(velocity));
    }

    /** Gets the output of the PID controller after adding the feedforward output using the 
     * controller velocity and acceleration.
     * @param currentPoint The current position of the controller in meters or radians.
     * @param targetPoint The target position of the controller in meters or radians.
     * @param velocity The target velocity of the controller in m/s or rad/s.
     * @param acceleration target The acceleration of the controller in m/s^2 or rad/s^2.
     * @return The PID controller output with applied feedforward.
    */
    public double getAcceleratingOutputWithFeedForward(double currentPoint, double targetPoint, double velocity, double acceleration)
    {
        return (this.calculate(currentPoint, targetPoint) + getAcceleratingFeedForward(velocity, acceleration));
    }

    /** Gets the output of the PID controller after adding the feedforward output using the 
     * controller velocity and acceleration.
     * @param currentPoint The current position of the controller in meters or radians.
     * @param targetProfile The target profile state object containing the target position and
     * velocity.
     * @param velocity The target velocity of the controller in m/s or rad/s.
     * @param acceleration The target acceleration of the controller in m/s^2 or rad/s^2.
     * @return The PID controller output with applied feedforward.
    */
    public double getAcceleratingOutputWithFeedForward(double currentPoint, TrapezoidProfile.State targetProfile, double velocity, double acceleration)
    {
        return (this.calculate(currentPoint, targetProfile) + getAcceleratingFeedForward(velocity, acceleration));
    }

    /** Calculates the difference between the current position and target position.
     * This overriden method uses the the targetprofileState from the constructor. If no target profile 
     * state was included, assume the target state is the current state.
     * @param currentPos The current position of the motor (encoder reading) in meters or radians.
     * @return The distance to travel as the next PID controller output in meters or radians.
     */
    @Override
    public double calculate(double currentPos)
    {
        return this.calculate(currentPos, targetProfileState);
    }

    /** Calculates the difference between the current position and target position using the constraints
     * from the constructor. This overriden method uses the Trapezoid Profile constraints as well as the
     * targetprofileState from the constructor. If no target profile state was included, assume the 
     * target state is the current state.
     * @param currentPos The current position of the motor (encoder reading) in meters or radians.
     * @return The distance to travel as the next PID controller output in meters or radians.
     */
    public double calculateWithConstraints(double currentPos)
    {
        return this.calculate(currentPos, targetProfileState, constraints);
    }




    //Methods to set the current Position and Velocity of the PID controller.




     /**Replaces the current state with the new one (newCurrentState argument). This method is 
     * functionaly the same as the reset method with the same parameters in the super class 
     * ProfiledPIDController, however, it was renamed and descripted to more accuratley explain the 
     * method's function. 
     * @param newCurrentState The object containing the current position (meters or radians) and 
     * velocity (m/s or rad/s) of the motor (encoder readings.)
     */
    public void setPositionAndVelocity(TrapezoidProfile.State newCurrentState)
    {
        //replace the existing object with a new one.
        this.reset(newCurrentState);
    }

    /**Replaces the current state with the new one. This method is functionaly the same as the reset 
     * method with the same parameters in the super class ProfiledPIDController, however, it was renamed 
     * and descripted to more accuratley explain the method's function. 
     * @param newCurrentPosition The current position of the motor (encoder reading) in meters or radians.
     * @param newCurrentVelocity The current velocity of the motor (encoder reading) in m/s or rad/s.
     */
    public void setPositionAndVelocity(double newCurrentPosition, double newCurrentVelocity)
    {
        //replace the existing values with the new ones.
        this.reset(newCurrentPosition, newCurrentVelocity);
    }




    //Methods to apply various values and settings to the PIDControllers while returning their output.




    /** Applies only the essential values to the PID controller and gets the calculated output of the 
     * controller. Ensure the units used for parameters match that ofthe ones used in the constructor 
     * for this object.
     * @param isContinuous If the motor is continuous (true) or not (false.) Making a controller 
     * continuous means that once it reaches a maximum or minimum value and tries to exceed either, 
     * the output jumps from the max to the min or vice versa instead of exceeding those values. This 
     * setup will only work if the units for this object are in radians, since the radial bounds are 
     * negative pie and pie.
     * @param isMeters If the type of units used for this object is meters (true) or radians (false.)
     * This is used to set a small tolerance since the PID controller would oscilate indefinatley 
     * otherwise. To set your own tolerance, use the setTolerance() method from the super class
     * ProfiledPIDController.
     * @param currentPoint The current position of the controller, usually fetched by the encoder and
     * converted to either meters or radians.
     * @param targetPoint the target position of the controller in either meters or radians.
     * @return The calculated output of the controller obtained from the difference of the current position
     * and the target position.
     * */
    public double applyBasicPID(
        boolean isContinuous, 
        boolean isMeters,
        double currentPoint,
        double targetPoint)
    {
       //apply the current position and velocity to the controller
       this.setPositionAndVelocity(currentProfileState);
       //if the controller is continuous, make it continuous at radial bounds negative pie and pie.
       if (isContinuous)
       {
           this.enableContinuousInput(-Math.PI, Math.PI);
       }
       //set the error position tolerance of the controller
       if(isMeters)
       {
        //set the tolerance to 0.0127 meters (about half an inch)
            this.setTolerance(0.0127);
       }
       else
       {
            //set the tolerance to 0.0873 radians (about 5 degrees)
            this.setTolerance(0.0873);
       }

       //if the setpoint or any point within the tolerance of it has been reached, 
       //don't worry about calculating, otherwise find the next output.
       if(this.atSetpoint())
       {
           return 0.0;
       }
       else
       {
           return this.calculate(currentPoint, targetPoint);
       }
    }

    /** Applies all essential and advanced values to the PID controller to maximize the accuracy of the 
     * controller and gets the calculated output of the controller. Ensure the units used for parameters
     * match that ofthe ones used in the constructor for this object.
     * @param isContinuous If the motor is continuous (true) or not (false.) Making a controller 
     * continuous means that once it reaches a maximum or minimum value and tries to exceed either, 
     * the output jumps from the max to the min or vice versa instead of exceeding those values. This 
     * setup will only work if the units for this object are in radians, since the radial bounds are 
     * negative pie and pie.
     * @param currentPoint The current position of the controller, usually fetched by the encoder and
     * converted to either meters or radians.
     * @param targetPoint the target position of the controller in either meters or radians.
     * @param errorPositionTolerance the distance in meters or radians in which the controller will 
     * consider itself at the setpoint if it is within this distance from the actual setpoint.
     * @param kS The static volts gain feedforward value to apply to the controller to reduce voltage 
     * output error.
     * @param kV The velocity gain feedforward value to apply to the controller to reduce velocity
     * output error.
     * @param kA The acceleration gain feedforward value to apply to the controller to reduce acceleration
     * output error.
     * @param currentVelocity The current velocity of the controller in m/s or rad/s (usually fetched 
     * from encoder.)
     * @param targetVelocity The target velocity for the controller to reach in m/s or rad/s. If the controller
     * should speed up, the max speed used for the TrapezoidProfile.Constraints object can be used to increase
     * the target output speed, or another target velocity can be used. If the controller should slow to 
     * a stop, this value can be zero.
     * @param targetAcceleration The target acceleration for the controller to reach in m/s^2 or rad/s^2. If 
     * the controller should speed up, the max acceleration used for the TrapezoidProfile.Constraints object 
     * can be used to speed it up, or another target velocity can be used. If the controller should 
     * slow to a stop, this value can be zero.
     * @return The calculated output of the controller obtained from the difference of the current position
     * and the target position.
     * */
    public double applyAllPID(
        boolean isContinuous,
        double currentPoint,
        double targetPoint,
        double errorPositionTolerance,
        double kS,
        double kV,
        double kA,
        double currentVelocity,
        double targetVelocity,
        double targetAcceleration
        )
    {
        currentProfileState = new TrapezoidProfile.State(currentPoint, currentVelocity);
        //set the feedforward values
        this.setNewFeedForward(kS, kV, kA);
        //apply the current position and velocity to the controller
        this.setPositionAndVelocity(currentProfileState);
        //if the controller is continuous, make it continuous at radial bounds negative pie and pie.
        if (isContinuous)
        {
            this.enableContinuousInput(-Math.PI, Math.PI);
        }
        //set the error position tolerance of the controller
        this.setTolerance(errorPositionTolerance);

        //if the setpoint or any point within the tolerance of it has been reached, 
        //don't worry about calculating, otherwise find the next output.
        if(this.atSetpoint())
        {
            return 0.0;
        }
        else
        {
            return getAcceleratingOutputWithFeedForward(currentPoint, targetPoint, targetVelocity, targetAcceleration);
        }
    }

     /** Applies all essential and advanced values to the PID controller (except accerleation gains and 
     * target) to maximize the accuracy of the controller and gets the calculated output of the 
     *controller. Ensure the units used for parameters match that ofthe ones used in the constructor 
     *for this object.
     * @param isContinuous If the motor is continuous (true) or not (false.) Making a controller 
     * continuous means that once it reaches a maximum or minimum value and tries to exceed either, 
     * the output jumps from the max to the min or vice versa instead of exceeding those values. This 
     * setup will only work if the units for this object are in radians, since the radial bounds are 
     * negative pie and pie.
     * @param currentPoint The current position of the controller, usually fetched by the encoder and
     * converted to either meters or radians.
     * @param targetPoint the target position of the controller in either meters or radians.
     * @param errorPositionTolerance the distance in meters or radians in which the controller will 
     * consider itself at the setpoint if it is within this distance from the actual setpoint.
     * @param kS The static volts gain feedforward value to apply to the controller to reduce voltage 
     * output error.
     * @param kV The velocity gain feedforward value to apply to the controller to reduce velocity
     * output error.
     * @param currentVelocity The current velocity of the controller in m/s or rad/s (usually fetched 
     * from encoder.)
     * @param targetVelocity The target velocity for the controller to reach in m/s or rad/s. If the controller
     * should speed up, the max speed used for the TrapezoidProfile.Constraints object can be used to increase
     * the target output speed, or another target velocity can be used. If the controller should slow to 
     * a stop, this value can be zero.
     * @return The calculated output of the controller obtained from the difference of the current position
     * and the target position.
     * */
    public double applyAllPID(
        boolean isContinuous,
        double currentPoint,
        double targetPoint,
        double errorPositionTolerance,
        double kS,
        double kV,
        double currentVelocity,
        double targetVelocity
        )
    {
        currentProfileState = new TrapezoidProfile.State(currentPoint, currentVelocity);
        //set the feedforward values
        this.setNewFeedForward(kS, kV, 0.0);
        //apply the current position and velocity to the controller
        this.setPositionAndVelocity(currentProfileState);
        //if the controller is continuous, make it continuous at radial bounds negative pie and pie.
        if (isContinuous)
        {
            this.enableContinuousInput(-Math.PI, Math.PI);
        }
        //set the error position tolerance of the controller
        this.setTolerance(errorPositionTolerance);

        //if the setpoint or any point within the tolerance of it has been reached, 
        //don't worry about calculating, otherwise find the next output.
        if(this.atSetpoint())
        {
            return 0.0;
        }
        else
        {
            return getConstantOutputWithFeedForward(currentPoint, targetPoint, currentVelocity);
        }
    }
}
