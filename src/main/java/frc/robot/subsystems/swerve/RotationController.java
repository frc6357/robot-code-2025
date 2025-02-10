package frc.robot.subsystems.swerve;

import static frc.robot.Konstants.SwerveConstants.kDHoldController;
import static frc.robot.Konstants.SwerveConstants.kDRotationController;
import static frc.robot.Konstants.SwerveConstants.kIHoldController;
import static frc.robot.Konstants.SwerveConstants.kIRotationController;
import static frc.robot.Konstants.SwerveConstants.kMaxAngularAcceleration;
import static frc.robot.Konstants.SwerveConstants.kMaxAngularVelocity;
import static frc.robot.Konstants.SwerveConstants.kPHoldController;
import static frc.robot.Konstants.SwerveConstants.kPRotationController;
import static frc.robot.Konstants.SwerveConstants.kRotationToleranceRadians;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;


// RotationController class provided by FRC 3847 Spectrum

/**
 * Uses a profiled PID Controller to quickly turn the robot to a specified angle. Once the robot is
 * within a certain tolerance of the goal angle, a PID controller is used to hold the robot at that
 * angle.
 */
public class RotationController {
    SK25Swerve swerve;
    SwerveConstantsConfigurator config;
    ProfiledPIDController controller;
    PIDController holdController;
    Constraints constraints;

    double calculatedValue = 0;

    /** Creates a new rotation controller object.*/
    public RotationController(SwerveConstantsConfigurator config) {
        this.config = config;
        constraints =
                //apply a velocity and acceleration limit on the controller output.
                new Constraints(kMaxAngularVelocity, kMaxAngularAcceleration);
        controller =
                new ProfiledPIDController(
                        kPRotationController,
                        kIRotationController,
                        kDRotationController,
                        constraints);

        //Allow the controller to wrap its angular output. In other words, don't allow 
        //the radian values to go above or below Pie or -Pie. Instead, jump from one 
        //endpoint to the other to never exit the radial bounds.
        controller.enableContinuousInput(-Math.PI, Math.PI);
        //Allow the controller to consider its target reach if it is within a specified 
        //distance of the target.
        controller.setTolerance(kRotationToleranceRadians);

        // Hold controller is standard PID
        holdController =
                new PIDController(
                        kPHoldController,
                        kIHoldController,
                        kDHoldController);

        //Allow the controller to wrap its angular output. In other words, don't allow 
        //the radian values to go above or below Pie or -Pie. Instead, jump from one 
        //endpoint to the other to never exit the radial bounds.
        holdController.enableContinuousInput(-Math.PI, Math.PI);
        //Allow the controller to consider its target reach if it is within a specified 
        //distance of the target.
        holdController.setTolerance(
                kRotationToleranceRadians / 2); // Half the tolerance of turn controller
    }

    /** Finds the value of the PID controllers next ouput, aka the remaining distance to cover from 
     * the current position and the target position. 
     * @param goalRadians The the target of the rotation controller in radians.
     * @param currentRadians The current position of the rotation controller in radians.
     * @param isHoldController If the controller is a hold controller (true) or regular rotation 
     * controller (false).
     * @return The calculated value output of the rotation controller. If the setpoint has been reached 
     * by a rotation controller (not a hold controller), including any distance within the position 
     * tolerance constant, an output of zero is returned.*/
    public double calculate(double goalRadians, double currentRadians, boolean isHoldController) {
        double measurement = currentRadians;
        calculatedValue = controller.calculate(measurement, goalRadians);

        if (atSetpoint()) {
            if (isHoldController) {
                calculatedValue = calculateHold(goalRadians, currentRadians);
                return calculatedValue;
            }
            calculatedValue = 0;
            return calculatedValue;
        } else {
            return calculatedValue;
        }
    }

    public double calculate(double goalRadians, double currentRadians) {
        return calculate(goalRadians, currentRadians, false);
    }

    public double calculateHold(double goalRadians, double currentRadians) {
        calculatedValue = holdController.calculate(currentRadians, goalRadians);
        return calculatedValue;
    }

    public boolean atSetpoint() {
        return controller.atSetpoint();
    }

    public void reset(double currentRadians) {
        controller.reset(currentRadians);
        holdController.reset();
    }

    public void updatePID(double kP, double kI, double kD) {
        controller.setPID(kP, kI, kD);
    }
}