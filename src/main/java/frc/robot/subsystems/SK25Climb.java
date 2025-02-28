//Subsystem Essentials
package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import frc.robot.preferences.Pref;
import frc.robot.preferences.SKPreferences;

import static edu.wpi.first.units.Units.Rotations;
import static frc.robot.Konstants.ClimbConstants.*;
import static frc.robot.Ports.ClimbPorts.*;
import static edu.wpi.first.units.Units.Rotation;
import static edu.wpi.first.units.Units.Volts;
import static frc.robot.Konstants.ClimbConstants.kClimbD;
import static frc.robot.Konstants.ClimbConstants.kClimbI;
import static frc.robot.Konstants.ClimbConstants.kClimbP;
import static frc.robot.Konstants.ClimbConstants.kVolts;
import static frc.robot.Ports.ClimbPorts.kClimbMotor;

import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.hardware.core.CoreCANcoder;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.swerve.utility.PhoenixPIDController;

import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Angle;
//SmartDashboard Import
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class SK25Climb extends SubsystemBase 
{
   //Declarations
   TalonFX motor;

   PhoenixPIDController climbPID;
   Slot0Configs climbPID0;

   TalonFXConfiguration motorConfig;

   double target;
   boolean atTarget;
   double timestamp;
   
   double motorCurrentPosition;
   PositionVoltage why;

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
      motor = new TalonFX(kClimbMotor.ID, kClimbMotor.bus);
      climbPID = new PhoenixPIDController(kClimbP, kClimbI, kClimbD);
      //  climbPID0 = new Slot0Configs()
      //    .withKP(climbkPPref.get())
      //    .withKI(climbkIPref.get())
      //    .withKD(climbkDPref.get())
      //    .withGravityType(GravityTypeValue.Arm_Cosine); // Gravity based feedforwards for an arm; bases output on motor position
      //  motorConfig = new TalonFXConfiguration()
      //    .withSlot0(climbPID0);

      motorConfig = new TalonFXConfiguration();
      motorConfig.MotorOutput.withNeutralMode(NeutralModeValue.Brake);

      //  why = new PositionVoltage(0).withSlot(0);
      //  TalonFXConfiguration configar = new TalonFXConfiguration();
      //  Slot0Configs configs = new Slot0Configs();
      //  configs.kP = kClimbP;
      //  configs.kI = kClimbI;
      //  configs.kD = kClimbD;
      //  configar.Voltage.withPeakForwardVoltage(Volts.of(0))
      //    .withPeakReverseVoltage(Volts.of(-0));

       motorCurrentPosition = 0.0;
      // motorTargetPosition = 0.0;



      // climbPID.reset();
      // climbPID.setTolerance(kClimbPositionTolerance);
      // motor.setPosition(0.0);
      //motor.getConfigurator().apply(configs);
   }

   //If we Eyeball
   public void runMotor(double speed) {
     // motor.setControl(new VelocityVoltage(speed));
    // motor.setVoltage(volts);
     motor.set(speed);
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

   //If we don't Eyeball
   public void setSetpoint(Angle setpoint) {
      //motor.setControl(why.withPosition(10));
      // motor.setPosition(setpoint);
     // motor.setVoltage(3);
   }

   public void stop() {
      motor.stopMotor();
   }

   // public void setPosition(double pos) {
   //    target = pos;
   //    motor.setControl(new PositionVoltage(target));
   // }

   // public boolean isAtTargetPosition() {
   //    atTarget = motor.getPosition().getValue().in(Rotations) == target;
   //    // boolean slow = motor.getVelocity().getValueAsDouble() < 1;
   //    return atTarget;
   // }

   @Override
   public void periodic() {
      timestamp += Robot.kDefaultPeriod;
     motorCurrentPosition = getMotorPosition(); //why is this here?
     //motorCurrentPosition =  motor.getPosition().getValue().in(Rotation);
     SmartDashboard.putNumber("Velocity (RpMs)", motorCurrentPosition);
   }
}