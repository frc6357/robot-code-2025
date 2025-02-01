
package frc.robot.subsystems;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.ClosedLoopSlot;


import edu.wpi.first.wpilibj2.command.SubsystemBase;


public class AlgaePickup extends SubsystemBase {


    //Speed that m_roller moves at
    double rollingSpeed=.1;

    //Speed that m_lifter moves at
    double liftSpeed = .1;

    //position it moves to when collecting algae
    double lowerPos = 1;

    //starting position. Goes to after releasing algae
    double upperPos = 0;

    //starting position. Goes to after releasing algae
    double returnPos = 0;

    SparkClosedLoopController m_PID;
    //changes direction of algae arm.
    SparkMax m_lifter;

    // Spins to grab or release algae
    SparkMax m_roller; 

    //encoder for m_lifter
    private AbsoluteEncoder m_encoder;

    
    private SparkMaxConfig motorConfig;
    public AlgaePickup(){
        m_lifter = new SparkMax(40, MotorType.kBrushless);
        m_roller = new SparkMax(41, MotorType.kBrushless);
        m_encoder = m_lifter.getAbsoluteEncoder();
        

        m_PID=m_lifter.getClosedLoopController();

        /*
        * Create a new SPARK MAX configuration object. This will store the
        * configuration parameters for the SPARK MAX that we will set below.
        */
        motorConfig = new SparkMaxConfig();

       

        motorConfig.closedLoop
            .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
            // Set PID values for position control. We don't need to pass a closed loop
            // slot, as it will default to slot 0.
            .p(0.1)
            .i(0)
            .d(0)
            .outputRange(-1, 1);
    }

    public void lower(){
        //Triggered when button 1 is pressed.

        //Sets target position for PID.
        m_PID.setReference(lowerPos, ControlType.kPosition, ClosedLoopSlot.kSlot0);
        /*
        if(m_PID.atSetpoint())
        {
            stopli();
            stopro();
        }
        else {
            m_roller.set(rollingSpeed);
            m_lifter.set(liftSpeed);
        }
         */
    }

    public void upper(){
        //Triggered when button 1 released

        //sets target position for PID
        m_PID.setReference(upperPos, ControlType.kPosition, ClosedLoopSlot.kSlot0);
        /*
        if(m_PID.atSetpoint()){
            //if it hasn't reached upper position yet, keep moving
            m_lifter.set(-liftSpeed);
            return false;
        }
        else{
            //stops apon reaching upper position
            stopli();

            //if true is returned, command knows to stop
            return true;
        }
        */
    }

    public void release(){
        //Spins the roller backwards when button 2 pressed. (stops when returns function triggered(button 2 released))
        m_roller.set(-rollingSpeed);
    }
    public void grab(){
        m_roller.set(-rollingSpeed);
    }

    public boolean returns(){
        //Triggered after button 2 released
        stopro();
        if (m_encoder.getPosition()>=returnPos){
            m_lifter.set(-liftSpeed);
            return false;
        }
        else{
            stopli();

            //if true is returned, command knows to stop
            return true;
        }
    }

    public void stopli(){
       //Stops the lifter
        m_lifter.set(0);
    }

    public void stopro(){
        //Stops the roller
        m_roller.set(0);
    }

    public void teleopPeriodic(){
    }
    public void periodic(){
    }
    public void testInit(){
    }
    public void testPeriodic(){
    }
}
