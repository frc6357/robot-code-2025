package frc.robot.subsystems;


import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkAbsoluteEncoder;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import static frc.robot.Ports.EndEffectorPorts.kArmMotor;
//import static frc.robot.Ports.EndEffectorPorts.kRollerMotor;
import static frc.robot.Konstants.EndEffectorConstants.kArmSpeed;
//import static frc.robot.Konstants.EndEffectorConstants.kRollerSpeed;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;





public class EndEffectorV1  extends SubsystemBase{

    //declare a motor object of type CANSParkFlex
    //SparkFlex rollerMotor;
    SparkFlex armMotor;
    SparkFlexConfig armConfig;

    SparkClosedLoopController mPID;
    double mtargetPosition;
    double mcurrentPosition;

    SparkAbsoluteEncoder mEncoder;

    ArmFeedforward  armFeedforward;

    //constructor
    public EndEffectorV1()
    {
        

        
        //initialize the new motor object with its motor ID and type
        //rollerMotor = new SparkFlex(kRollerMotor.ID, MotorType.kBrushless);
        armMotor = new SparkFlex(kArmMotor.ID, MotorType.kBrushless);

        armConfig = new SparkFlexConfig();
        
        armConfig.absoluteEncoder 
            .positionConversionFactor(1)
            .velocityConversionFactor(1);

        armConfig.closedLoop
            .feedbackSensor(FeedbackSensor.kAbsoluteEncoder)
            .p(0.1)
            .i(0)
            .d(0)
            .outputRange(-1, 1)
            .p(0, ClosedLoopSlot.kSlot1)
            .i(0, ClosedLoopSlot.kSlot1)
            .d(0, ClosedLoopSlot.kSlot1)
            .velocityFF(1.0/5767, ClosedLoopSlot.kSlot1)
            .outputRange(-1, 1, ClosedLoopSlot.kSlot1);

        mPID = armMotor.getClosedLoopController();

        mEncoder = armMotor.getAbsoluteEncoder();

        armFeedforward = new ArmFeedforward(0.22,0.58, 0.10, 0.01 );

       armMotor.configure(armConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);

    }

    //runs the motor
    //public void runRoller()
    {
        //rollerMotor.set(kRollerSpeed);
    }

    public void runArm()
    {
        armMotor.set(kArmSpeed);
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
