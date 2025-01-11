package frc.robot; //Add .subsystems when merging

import com.revrobotics.CANSparkFlex;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
//import static frc.robot.Konstants.ElevatorConstants.*;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

//import static frc.robot.Ports.elevatorPorts.*;

public class SK25Elevator extends SubsystemBase
{
    //Create memory motor objects
    CANSparkFlex motor;

    //Create memory PID object
    PIDController PID;

    double currentPosition;

    double targetPosition;

    RelativeEncoder encoder;

    //Constructor for public command access
    public SK25Elevator()
    {
        //Initialize motor objects
        //PID = new PIDController(rightClimb.kP, rightClimb.kI, rightClimb.kD);
        PID.setSetpoint(0.0);

        //motor = new SparkFlex(kRightClimbMotor.ID, MotorType.kBrushless);

        motor.setInverted(false);
        encoder = motor.getEncoder();
        
        RelativeEncoder encoder = motor.getEncoder();
        //encoderR.setPositionConversionFactor(climbConversion);
        //resetPosition(0.0);

        targetPosition = 0.0;
        currentPosition = 0.0;
    }

    public void setElevatorPos(double location)
    {
        targetPosition = location;
        PID.setSetpoint(location);
    }

    public void runElevator(double speed)
    {
        motor.set(speed);
    }


    public double getElevatorPos()
    {
        return encoder.getPosition();
    }

    public double getTargetPosition()
    {
        return targetPosition;
    }

    /*
    public boolean isRightAtTargetPosition()
    {
        //return Math.abs(getRightPosition() - getRightTargetPosition()) < kPositionTolerance;
    }

    public boolean isLeftAtTargetPosition()
    {
        //return Math.abs(getLeftPosition() - getLeftTargetPosition()) < kPositionTolerance;
    }

    public void resetElevatorPos(double position){
        encoder.setPosition(position);
    }
    */

    public void stopElevator()
    {
        motor.stopMotor();
    }

    @Override
    public void periodic()
    {
        
        //double r_current_position = getRightPosition();
        // double r_target_position = getRightTargetPosition();

        //double l_current_position = getLeftPosition();
        // double l_target_position = getLeftTargetPosition();

        // // Calculates motor speed and puts it within operating range
        // double rSpeed = MathUtil.clamp(rPID.calculate(r_current_position), kClimbMotorMinOutput, kClimbMotorMaxOutput);
        // motorR.set(rSpeed); 

        // // Calculates motor speed and puts it within operating range
        // double lSpeed = MathUtil.clamp(lPID.calculate(l_current_position), kClimbMotorMinOutput, kClimbMotorMaxOutput);
        // motorL.set(lSpeed); 

        //SmartDashboard.putNumber("Right Current Position", r_current_position);
        // SmartDashboard.putNumber("Right Target Position", r_target_position);
        // SmartDashboard.putBoolean("Right Arm at Setpoint", isRightAtTargetPosition());

        //SmartDashboard.putNumber("Left Current Position", l_current_position);
        // SmartDashboard.putNumber("Left Target Position", l_target_position);
        // SmartDashboard.putBoolean("Left Arm at Setpoint", isLeftAtTargetPosition());
    }
}