package frc.robot.subsystems;


import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkAbsoluteEncoder;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import static frc.robot.Ports.EndEffectorPorts.kArmMotor;
import static frc.robot.Konstants.EndEffectorConstants.armAngleTolerance;
//import static frc.robot.Ports.EndEffectorPorts.kRollerMotor;
import static frc.robot.Konstants.EndEffectorConstants.kArmSpeed;
//import static frc.robot.Konstants.EndEffectorConstants.kRollerSpeed;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;





public class EndEffectorV1  extends SubsystemBase{

    //declare a motor object of type CANSparkMax
    //SparkMax rollerMotor;
    SparkMax armMotor;
    SparkMaxConfig armConfig;

    SparkClosedLoopController mPID;
    double mtargetPosition;
    double mcurrentPosition;

    public SparkAbsoluteEncoder mEncoder;

    ArmFeedforward  armFeedforward;

    double armTargetAngle;

    

    //constructor
    public EndEffectorV1()
    {
        

        
        //initialize the new motor object with its motor ID and type
        //rollerMotor = new SparkMax(kRollerMotor.ID, MotorType.kBrushless);
        armMotor = new SparkMax(kArmMotor.ID, MotorType.kBrushless);

        armConfig = new SparkMaxConfig();
        
        armConfig.absoluteEncoder 
            .positionConversionFactor(1)
            .velocityConversionFactor(1)
            .setSparkMaxDataPortConfig();

        armConfig.closedLoop
            .feedbackSensor(FeedbackSensor.kAbsoluteEncoder)
            .p(3)
            .i(0)
            .d(1)
            .outputRange(-.1, .1);
            //.p(0, ClosedLoopSlot.kSlot1)
            //.i(0, ClosedLoopSlot.kSlot1)
            //.d(0, ClosedLoopSlot.kSlot1)
            //.velocityFF(1.0/5767, ClosedLoopSlot.kSlot1)
            //.outputRange(-1, 1, ClosedLoopSlot.kSlot1);

        armConfig
            .idleMode(IdleMode.kBrake)
            .smartCurrentLimit(30);

        mPID = armMotor.getClosedLoopController();
        

        mEncoder = armMotor.getAbsoluteEncoder();

        armFeedforward = new ArmFeedforward(0.22,0.58, 0.10, 0.01 );

       armMotor.configure(armConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);

       armTargetAngle = 0.0;

    }

    public void setTargetAngle(double angle)
    {
        armTargetAngle = angle;
        //Come back and change this, need fraction for Encoder Rotations in place of angle
        mPID.setReference(angle, ControlType.kPosition,ClosedLoopSlot.kSlot0 );

    }

    public double getArmPosition()
    {
        //Set conversion factor
        return mEncoder.getPosition();
    }

    public double getTargetArmPosition()
    {
       return armTargetAngle;

    }

    public boolean isArmAtTargetPosition()
    {
        return Math.abs( getTargetArmPosition() -getArmPosition()) < armAngleTolerance;
    }

    


    //runs the motor
    //public void runRoller()
    {
        //rollerMotor.set(kRollerSpeed);
    }

    public void runArm(double armspeed)
    {
        armMotor.set(armspeed);
    }

    //stops the motor
    //public void stopRoller()
    {
       // rollerMotor.stopMotor();
    }

    public void stopArm()
    {
        armMotor.stopMotor();
    }

    //occurs every 20 miliseconds, usually not tied to a command, binder, etc...
    public void periodic()
    {
    } 

    public void testInit()
    {
    }
    
    public void testPeriodic()
    {
    }
    
}
