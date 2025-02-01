package frc.robot.subsystems;

import com.revrobotics.spark.SparkFlex;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import static frc.robot.Konstants.ExampleConstants.kExampleSpeed;
import static frc.robot.Ports.ExamplePorts.kExampleMotor;
import com.revrobotics.spark.SparkLowLevel.MotorType;


public class EndEffectorV1  extends SubsystemBase{

    //declare a motor object of type CANSParkFlex
    SparkFlex motor;

    //constructor
    public EndEffectorV1()
    {
        //initialize the new motor object with its motor ID and type
        motor = new SparkFlex(kExampleMotor.ID, MotorType.kBrushless);
    }

    //runs the motor
    public void runMotor()
    {
        motor.set(kExampleSpeed);
    }

    //stops the motor
    public void stopMotor()
    {
        motor.stopMotor();
    }

    //occurs every 20 miliseconds, usually not tied to a command, binder, etc...
    public void periodic()
    {
    } 

    public void testInit()
    {
    }
    
    public void testPeriodic()
    {
    }
    
}
