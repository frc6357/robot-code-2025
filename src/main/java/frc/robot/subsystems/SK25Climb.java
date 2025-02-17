//Subsystem Essentials
package frc.robot.subsystems;

import static frc.robot.Konstants.ClimbConstants.climbKD;
import static frc.robot.Konstants.ClimbConstants.climbkI;
import static frc.robot.Konstants.ClimbConstants.climbkP;
import static frc.robot.Konstants.ClimbConstants.kClimbMinPosition;
import static frc.robot.Konstants.ClimbConstants.kCurrentLimit;
import static frc.robot.Konstants.ClimbConstants.kMaxAcceleration;
import static frc.robot.Konstants.ClimbConstants.kMaxSpeed;
import static frc.robot.Konstants.ClimbConstants.kPositionTolerance;
import static frc.robot.Ports.ClimbPorts.kClimbMotor;

//Encoder Import
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkClosedLoopController;
//SparkMax Motor Imports
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.*;
import com.revrobotics.spark.config.MAXMotionConfig.MAXMotionPositionMode;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

//SmartDashboard Import
//import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;


public class SK25Climb extends SubsystemBase 
{
  
   //Declarations
   SparkMax motor;

   SparkClosedLoopController climbPID;

   RelativeEncoder encoder;

   SparkMaxConfig config;
   MAXMotionConfig smartConfig;

   double motorCurrentPosition;
   double motorTargetPosition;

   //Constructor
   public SK25Climb() 
   {
       //Initializations
       motor = new SparkMax(kClimbMotor.ID, MotorType.kBrushless);
       climbPID = motor.getClosedLoopController();
       config = new SparkMaxConfig();
       smartConfig = new MAXMotionConfig();
       config.closedLoop
         .pid(climbkP, climbkI, climbKD);
       config.idleMode(IdleMode.kBrake);
       config.smartCurrentLimit(kCurrentLimit);
       config.closedLoop.maxMotion
         .maxVelocity(kMaxSpeed) //RpM
         .maxAcceleration(kMaxAcceleration) //RpMpS
         .allowedClosedLoopError(kPositionTolerance)
         .positionMode(MAXMotionPositionMode.kMAXMotionTrapezoidal);
       motor.configure(config, SparkBase.ResetMode.kResetSafeParameters, SparkBase.PersistMode.kPersistParameters);
       encoder = motor.getEncoder();
       encoder.setPosition(kClimbMinPosition);
       motorCurrentPosition = 0.0;
       motorTargetPosition = 0.0;

      // climbPID.setSetpoint(0.0);

   }

   //Retrieve motor's speed
   public double getMotorSpeed()
   {
        return encoder.getVelocity();
   }

   //Retrieve motor's position
   public double getMotorPosition() 
   {
     return encoder.getPosition();
   }

   //Setting setpoints
   public void setPoint (double setpoint) 
   {
     // climbPID.setReference(setpoint, SparkBase.ControlType.kPosition);
     climbPID.setReference(setpoint, SparkBase.ControlType.kMAXMotionPositionControl); 
     }

   //Running motor
   public void runMotor(double speed)
   {}

   public double getTargetPosition() {
      return motorTargetPosition;
   }

   //Check Motor is at Target Position
   public boolean isAtTargetPosition()
   {
      return Math.abs(getMotorPosition() - getTargetPosition()) < kPositionTolerance;
   }

   public void stop() {
      motor.stopMotor();
   }

   @Override
   public void periodic() {
      motorCurrentPosition = getMotorPosition();
   }
}