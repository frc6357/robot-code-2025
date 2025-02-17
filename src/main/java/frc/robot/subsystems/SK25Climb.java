//Subsystem Essentials
package frc.robot.subsystems;

import static frc.robot.Konstants.ClimbConstants.climbKD;
import static frc.robot.Konstants.ClimbConstants.climbkI;
import static frc.robot.Konstants.ClimbConstants.climbkP;
import static frc.robot.Konstants.ClimbConstants.kPositionTolerance;
import static frc.robot.Konstants.ClimbConstants.*;
import static frc.robot.Ports.ClimbPorts.kClimbMotor;

//Encoder Import
import com.revrobotics.RelativeEncoder;
//SparkMax Motor Imports
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;

//PIDController Import
import edu.wpi.first.math.controller.PIDController;
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

   double motorCurrentPosition;
   double motorTargetPosition;

   //Constructor
   public SK25Climb() 
   {
       //Initializations
       motor = new SparkMax(kClimbMotor.ID, MotorType.kBrushless);
       climbPID = motor.getClosedLoopController();
       config = new SparkMaxConfig();
       config.closedLoop
         .pid(climbkP, climbkI, climbKD);
       encoder = motor.getEncoder();
       motorCurrentPosition = 0.0;
       motorTargetPosition = 0.0;

      // climbPID.setSetpoint(0.0);
      
       //config.idleMode(IdleMode.kBrake);
       //motor.configure(config, SparkBase.ResetMode.kResetSafeParameters, SparkBase.PersistMode.kNoPersistParameters);
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
      //   climbPID.reset();
      //   climbPID.setTolerance(kPositionTolerance);
      //   motorTargetPosition = setpoint;
      //   climbPID.setSetpoint(setpoint);
        // encoder.setPositionConversionFactor(0.0);
      //  encoder.setPosition(MathUtil.clamp(setpoint, kClimbMaxPosition, kClimbMinPosition));
      //  climbPID.setSetpoint(setPoint);
      //  motor.set(climbPID.calculate(getMotorSpeed(), setPoint));
      //  motor.set(MathUtil.clamp(setPoint(), kClimbMaxPosition, kClimbMinPosition));
      climbPID.setReference(setpoint, SparkBase.ControlType.kPosition);
   }

   //Running motor
   public void runMotor(double speed)
   {
     // motor.set(speed);
     // motor.setPosition(climbPID.calculate(getMotorPosition(), motorTargetPosition));
    // setPoint());
   }

   public double getTargetPosition() {
      return motorTargetPosition;
   }

   //Check Motor is at Target Position
   public boolean isAtTargetPosition()
   {
      return Math.abs(getMotorPosition() - getTargetPosition()) < kPositionTolerance;
    // return climbPID.atSetpoint();
   }

   public void stop() {
      motor.stopMotor();
   }

   @Override
   public void periodic() {
      motorCurrentPosition = getMotorPosition();
   }
}