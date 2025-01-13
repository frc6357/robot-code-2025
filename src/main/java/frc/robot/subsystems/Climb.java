package frc.robot.subsystems;


import edu.wpi.first.wpilibj2.command.SubsystemBase;


import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;


public class Climb extends SubsystemBase {
  
   //Test Motors - Subject to Change
   SparkMax liftMotorL;
   SparkMax liftMotorR;
  
   //Motor Positions, Current and Target
   double liftMotorCurrentPositionL;
   double lifeMotorCurrentPositionR;
   double liftMotorTargetPositionL;
   double liftMotorTargetPositionR;


   //Encoders - Subject to Change
   RelativeEncoder liftEncoderL;
   RelativeEncoder liftEncoderR;


   //Constructor
   public Climb () {
       //Initializations
       //Motor IDs are temporary placeholders
       //liftMotorL = new SparkMax(kRightClimbMotor.ID, MotorType.kBrushless);
       //liftMotorR = new SparkMax(kLeftClimbMotor.ID, MotorType.kBrushless);
       liftEncoderL = liftMotorL.getEncoder();
       liftEncoderR = liftMotorR.getEncoder();


       //idk if the motors'll spin in the same or different directions - subject to change
       liftMotorL.setInverted(false);
       liftMotorR.setInverted(true);




   }


   public double getLeftLiftMotorPosition() {
           return liftEncoderL.getPosition();
   }


   public double getRightLiftMotorPosition() {
       return liftEncoderR.getPosition();
   }


   @Override
   public void periodic() {
       double r_lift_current_position = getLeftLiftMotorPosition();


       double l_life_current_position = getRightLiftMotorPosition();
   }
}



