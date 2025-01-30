package frc.robot.subsystems;

import static frc.robot.Konstants.ClimbConstants.*;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;


import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;


public class SK25Climb extends SubsystemBase {
  
   //Test Motors - Subject to Change
   SparkMax motor;
  
   PIDController testPID;
   //Motor Positions, Current and Target
   double motorCurrentPosition;
   double motorTargetPosition;



   //Encoders - Subject to Change
   RelativeEncoder encoder;


   //Constructor
   public SK25Climb() {
       //Initializations
       //Motor IDs are temporary placeholders
       motor = new SparkMax(kRightClimbMotorId, MotorType.kBrushless);
       testPID = new PIDController(pid.kP, pid.kI, pid.kD);
       encoder = motor.getEncoder();
   }


   public double getMotorPosition() {
        return encoder.getPosition();
   }

   public void setPoint (double setPoint) {
        testPID.reset();
        testPID.setTolerance(0);
        testPID.setSetpoint(testPID.calculate(getMotorPosition(), setPoint));
     //    if (!testPID.atSetpoint()) {
     //      runMotor(kSpeed);
     //    } 
   }

   public void runMotor(double speed)
   {
        motor.set(speed);
   }

   @Override
   public void periodic() {}
}

