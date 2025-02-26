//Subsystem Essentials
package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import static frc.robot.Konstants.ClimbConstants.*;
import static frc.robot.Ports.ClimbPorts.*;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.hardware.core.CoreCANcoder;
import com.ctre.phoenix6.swerve.utility.PhoenixPIDController;
//Encoder Import
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkClosedLoopController;

//SparkMax Motor Imports, and Configurations
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import    com.revrobotics.spark.config.*;
import com.revrobotics.spark.config.MAXMotionConfig.MAXMotionPositionMode;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

//SmartDashboard Import
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class SK25Climb extends SubsystemBase 
{
   //Declarations
   TalonFX motor;

   PhoenixPIDController climbPID;

  // CoreCANcoder encoder;

   TalonFXConfiguration config;
   
   // double motorCurrentPosition;
   // double motorTargetPosition;

   //Constructor
   public SK25Climb() 
   {
      //Initializations
       motor = new TalonFX(kClimbMotor.ID);
       climbPID = new PhoenixPIDController(kClimbP, kClimbI, kClimbD);
       config = new TalonFXConfiguration();
       //encoder = new CoreCANcoder(kClimbEncoderID);
       //encoder.setPosition(kClimbMinPosition);

       //motorCurrentPosition = 0.0;
      // motorTargetPosition = 0.0;

      climbPID.reset();
      climbPID.setTolerance(kClimbPositionTolerance);
   }

   public void runMotor(double speed) {
      motor.setControl(new VelocityVoltage(speed));
   }

   // public double getMotorSpeed() {
   //    return encoder.getVelocity();
   // }

   // public double getMotorPosition() {
   //    return encoder.getPosition();
   // }

   // public double getTargetPosition() {
   //    return motorTargetPosition;
   // }

   public void stop() {
      motor.stopMotor();
   }

   //Sets setpoint *and* runs motor
   // public void setPoint (double setpoint) {
   //    climbPID.setReference(setpoint, SparkBase.ControlType.kMAXMotionPositionControl);
   // }

   //Changes Speed to new Value
   // public void cambiarVelocidad(double targetSpeed) {
   //   config.closedLoop
   //    .pid(kClimbP, kClimbI, kClimbD);
   //    config.idleMode(IdleMode.kBrake);
   //    config.smartCurrentLimit(kClimbCurrentLimit);
   //    config.closedLoop.maxMotion
   //   .maxVelocity(targetSpeed) //RpM
   //   .maxAcceleration(kClimbMaxAcceleration) //RpMpS
   //   .allowedClosedLoopError(kClimbPositionTolerance)
   //   .positionMode(MAXMotionPositionMode.kMAXMotionTrapezoidal);
   //   motor.configure(config, SparkBase.ResetMode.kResetSafeParameters, SparkBase.PersistMode.kPersistParameters);
   // }

   //Check if Motor is at Target Position
   // public boolean isAtTargetPosition() {
   //    return Math.abs(getMotorPosition() - getTargetPosition()) <= kClimbPositionTolerance;
   // }

   @Override
   public void periodic() {
    //  motorCurrentPosition = getMotorPosition(); //why is this here?
     // SmartDashboard.putNumber("Velocity (RPMs)", getMotorSpeed());
   }
}