package frc.robot.utils;

import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.config.MAXMotionConfig.MAXMotionPositionMode;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;


/** This class is intended to provide a more conveineint way to use PID controllers on hardware 
 * manufactured by REV Robotics. It extends the Rev SparkClosedLoopController object, which allows
 * any methods callable on the parent class to work with this object as well. REV's api for creating 
 * and using PID controllers is more simple and intuitive than Phoenix 6 api or WPILib's notation, so
 * this class can be read through to learn the simple steps of coding a PID controller.
 * */
public class SKRevPIDController{

   public SparkBase motor;
   public SparkMaxConfig config;
   public SparkClosedLoopController PIDController;
    
    /** This class is intended to provide a more conveineint way to use PID controllers on hardware 
    * manufactured by REV Robotics. It extends the Rev SparkClosedLoopController object, which allows
    * any methods callable on the parent class to work with this object as well. REV's api for creating 
    * and using PID controllers is more simple and intuitive than Phoenix 6 api or WPILib's notation, so
    * this class can be read through to learn the simple steps of coding a PID controller.
    * @param motor The SparkBase motor for the PID controller to be applied to.
    * */
    public SKRevPIDController(SparkBase motor)
    {
        this.motor = motor;
        PIDController =  motor.getClosedLoopController();
        config = new SparkMaxConfig();
    }

    /** This method handles the application of only the essential values to use a PID controller. Note 
     * that this method assumes the controller should be position based and not velocity based.
     */
    public void applyBasicPID(
        double setpoint, 
        double P, 
        double I, 
        double D,
        double positionTolerance,
        boolean isInverted
        )
    {
        //Apply settings to config object.
        config
            //invert the motor if it should be inverted
            .inverted(isInverted)
            //idle mode determines the state of the motor in idle, kBrake puts the motor in brake mode when inactive.
            .idleMode(IdleMode.kBrake)
            //set the supply current limit of them motor in amps
            .smartCurrentLimit(100)
                //apply settings to the closedLoop object of the configs object
                .closedLoop
                    //set the P, I, and D gains for the PID controller.
                    .pid(P, I, D)
                        //apply settings to the maxMotion object of the closedLoop object.
                        .maxMotion
                            //set the error tolerance of the PID controller.
                            .allowedClosedLoopError(positionTolerance);

        //Apply settings to the motor object, including setting safe reset parameters and persist parameters.
        motor.configure(config, SparkBase.ResetMode.kResetSafeParameters, SparkBase.PersistMode.kPersistParameters);
        //Apply the setpoint and control type to the controller, which automatically enacts them on the motor.
        PIDController.setReference(setpoint, ControlType.kPosition);
    }


    public void applyAllPID(
        double setpoint, 
        double P, 
        double I, 
        double D,
        double maxMotorSpeed,
        double maxMotorAcceleration,
        double positionTolerance,
        ControlType controlType,
        boolean isInverted,
        boolean idleMode,
        int currentLimitAmps)
    {
        //Apply settings to config object.
        config
            //invert the motor if it should be inverted
            .inverted(isInverted)
            //idle mode determines the state of the motor in idle, kBrake puts the motor in brake mode when inactive.
            .idleMode(IdleMode.kBrake)
            //set the supply current limit of them motor in amps
            .smartCurrentLimit(currentLimitAmps)
                //apply settings to the closedLoop object of the configs object
                .closedLoop
                    //set the P, I, and D gains for the PID controller.
                    .pid(P, I, D)
                        //apply settings to the maxMotion object of the closedLoop object.
                        .maxMotion
                            //set the max velocity of the controller in Rpm
                            .maxVelocity(maxMotorSpeed) //RpM
                            //set the max acceleration of the controller in Rpmps 
                            .maxAcceleration(maxMotorAcceleration) //RpMpS
                            //set the error tolerance of the PID controller.
                            .allowedClosedLoopError(positionTolerance);
    
        //if (controlTpye)
            //set the controller to position mode.
            config.closedLoop.maxMotion.positionMode(MAXMotionPositionMode.kMAXMotionTrapezoidal);

        //Apply settings to the motor object, including setting safe reset parameters and persist parameters.
        motor.configure(config, SparkBase.ResetMode.kResetSafeParameters, SparkBase.PersistMode.kPersistParameters);
        //Apply the setpoint and control type to the controller, which automatically enacts them on the motor.
        PIDController.setReference(setpoint, controlType);
    }

    public void applyAllPIDFollower(
        double setpoint, 
        double P, 
        double I, 
        double D,
        double maxMotorSpeed,
        double maxMotorAcceleration,
        double positionTolerance,
        ControlType controlType,
        boolean isInverted,
        boolean idleMode,
        int currentLimitAmps,
        int leaderID)
    {
        //Apply settings to config object.
        config
            //invert the motor if it should be inverted
            .inverted(isInverted)
            //determine if the motor is a follower
            .follow(leaderID)
            //idle mode determines the state of the motor in idle, kBrake puts the motor in brake mode when inactive.
            .idleMode(IdleMode.kBrake)
            //set the supply current limit of them motor in amps
            .smartCurrentLimit(currentLimitAmps)
                //apply settings to the closedLoop object of the configs object
                .closedLoop
                    //set the P, I, and D gains for the PID controller.
                    .pid(P, I, D)
                        //apply settings to the maxMotion object of the closedLoop object.
                        .maxMotion
                            //set the max velocity of the controller in Rpm
                            .maxVelocity(maxMotorSpeed) //RpM
                            //set the max acceleration of the controller in Rpmps 
                            .maxAcceleration(maxMotorAcceleration) //RpMpS
                            //set the error tolerance of the PID controller.
                            .allowedClosedLoopError(positionTolerance)
                            //set the controller to position mode.
                            .positionMode(MAXMotionPositionMode.kMAXMotionTrapezoidal);

        //Apply settings to the motor object, including setting safe reset parameters and persist parameters.
        motor.configure(config, SparkBase.ResetMode.kResetSafeParameters, SparkBase.PersistMode.kPersistParameters);
        //Apply the setpoint and control type to the controller, which automatically enacts them on the motor.
        PIDController.setReference(setpoint, controlType);
    }
}
