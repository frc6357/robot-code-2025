package frc.robot.subsystems;

import static frc.robot.Konstants.EndEffectorConstants.kEndEffectorMotorMaxOutput;
import static frc.robot.Konstants.EndEffectorConstants.kEndEffetorMotorMinOutput;
import static frc.robot.Konstants.EndEffectorConstants.kArmTolerance;
import static frc.robot.Ports.EndEffectorPorts.kEndEffectorArmMotor;
import static frc.robot.Ports.EndEffectorPorts.kEndEffectorRollerMotor;
import static frc.robot.Konstants.EndEffectorConstants.kRollerSpeed;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;


import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkAbsoluteEncoder;
import com.revrobotics.spark.SparkRelativeEncoder;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;

public class EndEffectorV2 extends SubsystemBase
{
    final int motorRatio = 5;
    final int gear1Rotation = 40;
    final int gear2Rotation = 42;
    final int degrees = 360;

    SparkMax armMotor;
    SparkMaxConfig armConfig;

    SparkClosedLoopController mPID;
    double mtargetPosition;
    double mcurrentPosition;

    private SparkRelativeEncoder mEncoder;

    ArmFeedforward  armFeedforward;

    double armTargetAngle;

    public EndEffectorV2()
    {
        

        
        //initialize the new motor object with its motor ID and type
        //rollerMotor = new SparkMax(kRollerMotor.ID, MotorType.kBrushless);
        armMotor = new SparkMax(kEndEffectorArmMotor.ID, MotorType.kBrushless);

        armConfig = new SparkMaxConfig();
        
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
        
        armFeedforward = new ArmFeedforward(0.22,0.58, 0.10, 0.01 );

       armMotor.configure(armConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);

       armTargetAngle = 0.0;

    }


}