package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.controller.PIDController;

public class SwerveWheel {

    PIDController directionController;
    double currentDistance = 0.0;
    double targetDistance = 0.0;
    TalonFX driveMotor;

    public SwerveWheel(double P, double I, double D)
    {
        directionController = new PIDController(P, I, D);
    }

    public void setDirection(double setpoint)
    {
        directionController.reset();
        //allows the PID loop to take the smaller of the two errors, for example, traveling -90 degrees instead of 270.
        directionController.enableContinuousInput(-180, 180);
        //sets the acceptable error bound to which the controller will stop if reached
        directionController.setTolerance(0.2);
        //sets the target value for the PID controller
        directionController.setSetpoint(setpoint);
    }

    public void setSpeed(double speed)
    {
        driveMotor.set(speed);
    }
}
