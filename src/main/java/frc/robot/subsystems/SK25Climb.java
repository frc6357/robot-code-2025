//Subsystem Essentials
package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import frc.robot.preferences.Pref;
import frc.robot.preferences.SKPreferences;

import static edu.wpi.first.units.Units.Rotations;
import static frc.robot.Konstants.ClimbConstants.*;
import static frc.robot.Ports.ClimbPorts.*;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.hardware.core.CoreCANcoder;
import com.ctre.phoenix6.signals.GravityTypeValue;
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

import edu.wpi.first.units.measure.AngularVelocity;
//SmartDashboard Import
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class SK25Climb extends SubsystemBase 
{
   //Declarations
   TalonFX motor;

   PhoenixPIDController climbPID;
   Slot0Configs climbPID0;

  // CoreCANcoder encoder;

   TalonFXConfiguration motorConfig;

   double target;
   boolean atTarget;
   double timestamp;
   
   // double motorCurrentPosition;
   // double motorTargetPosition;

   Pref<Double> climbkPPref = SKPreferences.attach("climbP", 1.0)
      .onChange((newValue) -> climbPID.setP(newValue));
   Pref<Double> climbkIPref = SKPreferences.attach("climbI", 0.0)
      .onChange((newValue) -> climbPID.setI(newValue));
   Pref<Double> climbkDPref = SKPreferences.attach("climbD", 0.0)
      .onChange((newValue) -> climbPID.setI(newValue));

   //Constructor
   public SK25Climb() 
   {
      //Initializations
      timestamp = 0.0;
       motor = new TalonFX(kClimbMotor.ID);
       climbPID = new PhoenixPIDController(kClimbP, kClimbI, kClimbD);
       climbPID0 = new Slot0Configs()
         .withKP(climbkPPref.get())
         .withKI(climbkIPref.get())
         .withKD(climbkDPref.get())
         .withGravityType(GravityTypeValue.Arm_Cosine); // Gravity based feedforwards for an arm; bases output on motor position
       motorConfig = new TalonFXConfiguration()
         .withSlot0(climbPID0);

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

   public double getMotorSpeed() {
      return motor.getVelocity().getValueAsDouble(); // Rotations / sec
   }

   public double getMotorPosition() {
      return motor.getPosition().getValueAsDouble(); // Rotations
   }

   public double getTargetPosition() {
      return motor.getClosedLoopReference().getValueAsDouble();
   }

   public void stop() {
      motor.stopMotor();
   }

   public void setPosition(double pos) {
      target = pos;
      motor.setControl(new PositionVoltage(target));
   }

   public boolean isAtTargetPosition() {
      atTarget = motor.getPosition().getValue().in(Rotations) == target;
      // boolean slow = motor.getVelocity().getValueAsDouble() < 1;
      return atTarget;
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
      timestamp += Robot.kDefaultPeriod;
    //  motorCurrentPosition = getMotorPosition(); //why is this here?
     // SmartDashboard.putNumber("Velocity (RPMs)", getMotorSpeed());
   }
}