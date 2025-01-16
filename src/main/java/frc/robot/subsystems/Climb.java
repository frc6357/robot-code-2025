package frc.robot.subsystems;


import edu.wpi.first.wpilibj2.command.SubsystemBase;


import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;


public class Climb extends SubsystemBase {
  
   //Test Motors - Subject to Change
   SparkMax motorL;
   SparkMax motorR;
  
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
       //Motor IDs are temporary placeholders
       //motorL = new SparkMax(kRightClimbMotor.ID, MotorType.kBrushless);
       //motorR = new SparkMax(kLeftClimbMotor.ID, MotorType.kBrushless);
       encoderL = motorL.getEncoder();
       encoderR = motorR.getEncoder();


       //idk if the motors'll spin in the same or different directions - subject to change
       motorL.setInverted(false);
       motorR.setInverted(true);




   }


   public double getLeftMotorPosition() {
           return encoderL.getPosition();
   }


   public double getRightMotorPosition() {
       return encoderR.getPosition();
   }


   @Override
   public void periodic() {
       double r_current_position = getLeftMotorPosition();


       double l_current_position = getRightMotorPosition();
   }
}



