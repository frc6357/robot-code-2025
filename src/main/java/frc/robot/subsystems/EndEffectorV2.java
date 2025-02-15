package frc.robot.subsystems;

import static frc.robot.Konstants.EndEffectorConstants.kArmTolerance;
import static frc.robot.Ports.EndEffectorPorts.kEndEffectorArmMotor;
import static frc.robot.Ports.EndEffectorPorts.kEndEffectorRollerMotor;
import static frc.robot.Konstants.EndEffectorConstants.kRollerSpeed;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.RelativeEncoder;

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
    //change to 25 when done testing.
    final int motorRatio = 5;
    final int gear1Rotation = 40;
    final int gear2Rotation = 42;
    final int degrees = 360;

    SparkMax armMotor;
    SparkMax rollerMotor;
    SparkMaxConfig armConfig;

    SparkClosedLoopController mPID;
    double mTargetAngle;
    double mCurrentAngle;

    public RelativeEncoder mEncoder;

    ArmFeedforward  armFeedforward;

    double armTargetAngle;

    public EndEffectorV2()
    {
        

        
        //initialize the new motor object with its motor ID and type
        rollerMotor = new SparkMax(kEndEffectorRollerMotor.ID, MotorType.kBrushless);
        armMotor = new SparkMax(kEndEffectorArmMotor.ID, MotorType.kBrushless);

        armConfig = new SparkMaxConfig();
        
        armConfig.closedLoop
            .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
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

        mEncoder = armMotor.getEncoder();
        
        //armFeedforward = new ArmFeedforward(0.22,0.58, 0.10, 0.01 );

        armMotor.configure(armConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);

        mTargetAngle = 0.0;
        mCurrentAngle = 0.0;

        mEncoder.setPosition(0);
    }

    public void initialize()
    {

    }
        
    
    public void resetEncoder()
    {
        mEncoder.setPosition(0);
    }

    public void setTargetAngle(double angle)
    {
        mTargetAngle = angle;

        double motorRotations = (angle/degrees/gear2Rotation) * gear1Rotation * motorRatio;

        System.out.println("Motor " + motorRotations);
        System.out.println("Encoder " + mEncoder.getPosition());
        //Come back and change this, need fraction for Encoder Rotations in place of angle
        mPID.setReference(motorRotations, ControlType.kPosition,ClosedLoopSlot.kSlot0 );

    }

    public double getArmPosition()
    {
        //Set conversion factor
        double motorRotations = mEncoder.getPosition();
        double angle = (motorRotations * gear2Rotation * degrees) / motorRatio / gear1Rotation;
        return angle;
    }

    public double getTargetArmPosition()
    {
       return mTargetAngle;

    }

    public boolean isArmAtTargetPosition()
    {
        return Math.abs( getTargetArmPosition() -getArmPosition()) < kArmTolerance;
    }

     public void runRoller(double rollerspeed)
     {
        rollerMotor.set(rollerspeed);
    }

    public void runArm(double armspeed)
    {
        armMotor.set(armspeed);
    }

    //stops the motor
    public void stopRoller()
    {
        rollerMotor.stopMotor();
    }

    public void stopArm()
    {
        armMotor.stopMotor();
    }

    public void periodic()
    {

        
        /*if (SmartDashboard.getBoolean("Control Mode", false)) 
        {
            double targetVelocity = SmartDashboard.getNumber("Target Velocity", 0);
            mPID.setReference(targetVelocity, ControlType.kVelocity, ClosedLoopSlot.kSlot1);
        } 
        else 
        {
            double targetPosition = SmartDashboard.getNumber("Target Position", 0);
            mPID.setReference(targetPosition, ControlType.kPosition, ClosedLoopSlot.kSlot0);
        }

        SmartDashboard.putNumber("Actual Position", mEncoder.getPosition());
        SmartDashboard.putNumber("Actual Velocity", mEncoder.getVelocity());

        if (SmartDashboard.getBoolean("Reset Encoder", false)) 
        {
            SmartDashboard.putBoolean("Reset Encoder", false);
            mEncoder.setPosition(0);
        }
        
        double currentPosition = getArmPosition();
        double armTargetPosition = getTargetArmPosition();
        
        SmartDashboard.putNumber("Current Estimated Position", currentPosition);

        SmartDashboard.putNumber("Arm Target Position", armTargetPosition);
        SmartDashboard.putBoolean("Arm at Setpoint", isArmAtTargetPosition());*/
        
    }

    public void testPeriodic(){}
    public void testInit(){}

}