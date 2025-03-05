//Subsystem Essentials
package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import frc.robot.preferences.Pref;
import frc.robot.preferences.SKPreferences;

//import static edu.wpi.first.units.Units.Rotations;
import static frc.robot.Konstants.ClimbConstants.*;
//import static frc.robot.Ports.ClimbPorts.*;
//import static edu.wpi.first.units.Units.Rotation;
//import static edu.wpi.first.units.Units.Volts;
import static frc.robot.Konstants.ClimbConstants.kClimbD;
import static frc.robot.Konstants.ClimbConstants.kClimbI;
import static frc.robot.Konstants.ClimbConstants.kClimbP;
import static frc.robot.Ports.ClimbPorts.kClimbMotor;

//import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityDutyCycle;
import com.ctre.phoenix6.controls.compound.Diff_DutyCycleOut_Velocity;
//import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
//import com.ctre.phoenix6.hardware.core.CoreCANcoder;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
//import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.swerve.utility.PhoenixPIDController;

import edu.wpi.first.units.Units;
//import edu.wpi.first.units.measure.AngularVelocity;
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
   MotorOutputConfigs outputConfigs;

   double target;
   boolean atTarget;
   double timestamp;
   
   double motorCurrentPosition;
   PositionVoltage why;

   NeutralOut neutral = new NeutralOut();

   double motorTargetPosition;

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

      climbPID.reset();
      climbPID.setTolerance(kClimbPositionTolerance);

      motorConfig = new TalonFXConfiguration();
      motorConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;

       climbPID0 = new Slot0Configs()
         .withKP(climbkPPref.get())
         .withKI(climbkIPref.get())
         .withKD(climbkDPref.get())
         .withGravityType(GravityTypeValue.Arm_Cosine); // Gravity based feedforwards for an arm; bases output on motor position

      // climbPID0 = new Slot0Configs();
      //    climbPID0.kP = kClimbP;
      //    climbPID0.kI = kClimbI;
      //    climbPID0.kD = kClimbD;

         motorConfig.withSlot0(climbPID0);

   //Neutral Mode Attempts
      // motorConfig.MotorOutput.withNeutralMode(NeutralModeValue.Brake); //Neutral Mode
      // motor.setNeutralMode(NeutralModeValue.Brake);
      // outputConfigs.withNeutralMode(NeutralModeValue.Brake);

      //  why = new PositionVoltage(0).withSlot(0);
      //  TalonFXConfiguration configar = new TalonFXConfiguration();
      //  Slot0Configs configs = new Slot0Configs();
      //  configs.kP = kClimbP;
      //  configs.kI = kClimbI;
      //  configs.kD = kClimbD;
      //  configar.Voltage.withPeakForwardVoltage(Volts.of(0))
      //    .withPeakReverseVoltage(Volts.of(-0));

       motorCurrentPosition = kClimbMinPosition;
       motorTargetPosition = kClimbMaxPosition;

      //  motor.setPosition(kClimbMinPosition);
      //  motor.getConfigurator().apply(motorConfig);
   }

   public void setBrake() {
      motor.setControl(neutral);
   }
   //If we Eyeball
   public void runMotor(double speed) {
      // Sets a velocity to target via pid and supplies an average duty cycle in volts
     motor.setControl(new VelocityDutyCycle(speed).withFeedForward(3.0)); // FF in volts
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

   public boolean isAtTargetPosition() {
     // atTarget = motor.getPosition().getValue().in(Rotations) == target;
      // boolean slow = motor.getVelocity().getValueAsDouble() < 1;
      //return atTarget;
      return Math.abs(getTargetPosition() - getMotorPosition()) <= kClimbPositionTolerance;
   }

   @Override
   public void periodic() {
      timestamp += Robot.kDefaultPeriod;
      motorCurrentPosition = getMotorPosition(); 
     //motorCurrentPosition =  motor.getPosition().getValue().in(Rotation);
     SmartDashboard.putNumber("Velocity (RpMs)", getMotorSpeed());
   }
}