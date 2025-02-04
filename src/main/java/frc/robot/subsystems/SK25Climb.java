//Subsystem Essentials
package frc.robot.subsystems;

//Konstants (muy importante)
import static frc.robot.Konstants.ClimbConstants.kClimbMaxPosition;
import static frc.robot.Konstants.ClimbConstants.kClimbMinPosition;
import static frc.robot.Konstants.ClimbConstants.kPositionTolerance;
import static frc.robot.Konstants.ClimbConstants.pid;
import static frc.robot.Ports.ClimbPorts.*;

//Encoder Import
import com.revrobotics.RelativeEncoder;
//SparkMax Motor Imports
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.math.MathUtil;
//PIDController Import
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

//SmartDashboard Import
//import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;


public class SK25Climb extends SubsystemBase 
{
  
   //Declarations
   SparkMax motor;

   PIDController climbPID;

   RelativeEncoder encoder;

   SparkMaxConfig config;

   double motorCurrentPosition;
   double motorTargetPosition;

   //Constructor
   public SK25Climb() 
   {
       //Initializations
       motor = new SparkMax(kClimbMotor.ID, MotorType.kBrushless);
       climbPID = new PIDController(pid.kP, pid.kI, pid.kD);
       encoder = motor.getEncoder();
       motorCurrentPosition = 0.0;
       motorTargetPosition = 0.0;

       climbPID.setSetpoint(0.0);

       config.idleMode(IdleMode.kBrake);
       motor.configure(config, SparkBase.ResetMode.kResetSafeParameters, SparkBase.PersistMode.kNoPersistParameters);
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
        climbPID.reset();
        climbPID.setTolerance(0);
        motorTargetPosition = setpoint;
        climbPID.setSetpoint(setpoint);
        // encoder.setPositionConversionFactor(0.0);
      //  encoder.setPosition(MathUtil.clamp(setpoint, kClimbMaxPosition, kClimbMinPosition));
      //  climbPID.setSetpoint(setPoint);
      //  motor.set(climbPID.calculate(getMotorSpeed(), setPoint));
      //  motor.set(MathUtil.clamp(setPoint(), kClimbMaxPosition, kClimbMinPosition));
        
   }

   //Running motor
   public void runMotor(double speed)
   {
       motor.set(speed);
   }

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