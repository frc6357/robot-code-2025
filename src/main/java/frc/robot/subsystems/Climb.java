package frc.robot.subsystems;

import static frc.robot.Konstants.ClimbConstants.*;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;


public class Climb extends SubsystemBase {
  
   //Test Motors - Subject to Change
   SparkMax motorL;
   SparkMax motorR;
  
    PIDController testPID;
   //Motor Positions, Current and Target
   double motorCurrentPositionL;
   double motorCurrentPositionR;
   double motorTargetPositionL;
   double motorTargetPositionR;


   //Encoders - Subject to Change
   RelativeEncoder encoderL;
   RelativeEncoder encoderR;


   //Constructor
   public Climb () {
       //Initializations
       testPID = new PIDController(pid.kP, pid.kI, pid.kD);
       //Motor IDs are temporary placeholders
       //motorL = new SparkMax(kRightClimbMotor.ID, MotorType.kBrushless);
       //motorR = new SparkMax(kLeftClimbMotor.ID, MotorType.kBrushless);
       encoderL = motorL.getEncoder();
       encoderR = motorR.getEncoder();
   }


   public double getLeftMotorPosition() {
        return encoderL.getPosition();
   }

   public double getRightMotorPosition() {
       return encoderR.getPosition();
   }

   public void setPointL (double setPoint) {
        testPID.reset();
        testPID.setTolerance(0);
        motorL.set(testPID.calculate(getLeftMotorPosition(), setPoint));
   }

   public void setPointR (double setPoint) {
    testPID.reset();
    testPID.setTolerance(0);
    motorL.set(testPID.calculate(getRightMotorPosition(), setPoint));
}

   public void runLeftHook(double speed)
   {
       motorL.set(speed);
   }

   public void runRightHook(double speed)
   {
       motorR.set(speed);
   }


   @Override
   public void periodic() {}
}



