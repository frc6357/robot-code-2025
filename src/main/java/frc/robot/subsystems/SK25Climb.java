//Subsystem Essentials
package frc.robot.subsystems;

import static frc.robot.Konstants.ClimbConstants.kClimbD;
import static frc.robot.Konstants.ClimbConstants.kClimbI;
import static frc.robot.Konstants.ClimbConstants.kClimbMaxPosition;
import static frc.robot.Konstants.ClimbConstants.kClimbMinPosition;
import static frc.robot.Konstants.ClimbConstants.kClimbP;
import static frc.robot.Konstants.ClimbConstants.kClimbPositionTolerance;
import static frc.robot.Ports.ClimbPorts.kClimbMotor;

import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.swerve.utility.PhoenixPIDController;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import frc.robot.preferences.Pref;
import frc.robot.preferences.SKPreferences;

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

      //Brake Mode
      // motorConfig.MotorOutput.withNeutralMode(NeutralModeValue.Brake);

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
      if((getMotorPosition() > (kClimbMaxPosition - kClimbPositionTolerance)) && Math.signum(speed) > 0)
      {
         stop();
         return;
      }
      else
      { 
         // Sets a velocity to target via pid and supplies an average duty cycle in volts
         motor.setControl(new DutyCycleOut(speed));//.withFeedForward(3.0)); // FF in volts
      }

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
     SmartDashboard.putNumber("ClimbVelocity (RpMs)", getMotorSpeed());
     SmartDashboard.putNumber("ClimbPos", getMotorPosition());
   }
}