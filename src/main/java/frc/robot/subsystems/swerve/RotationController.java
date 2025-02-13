package frc.robot.subsystems.swerve;

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
    SwerveConfig config;
    ProfiledPIDController controller;
    PIDController holdController;
    Constraints constraints;

    double calculatedValue = 0;

    public RotationController(SwerveConfig config) {
        this.config = config;
        constraints =
                new Constraints(config.getMaxAngularVelocity(), config.getMaxAngularAcceleration());
        controller =
                new ProfiledPIDController(
                        config.getKPRotationController(),
                        config.getKIRotationController(),
                        config.getKDRotationController(),
                        constraints);

        // Effectively creates a unit circle of input for cocentric angles
        controller.enableContinuousInput(-Math.PI, Math.PI); 

        // Sets the tolerance for the difference between robot and goal angle
        // It is relatively looser than the HoldController tolerance to allow for smoother movement
        controller.setTolerance(config.getRotationTolerance()); 

        // Hold controller is standard PID
        holdController =
                new PIDController(
                        config.getKPHoldController(),
                        config.getKIHoldController(),
                        config.getKDHoldController());

        holdController.enableContinuousInput(-Math.PI, Math.PI);
        holdController.setTolerance(
                config.getRotationTolerance() / 2); // Half the error tolerance of turn controller
    }

    /**
     * Calculates the power needed to rotate the robot to a set point based on which type
     * of PID controller is used (Rotating vs. Holding Controller)
     * 
     * @param goalRadians      The goal rotation of the robot in radians
     * @param currentRadians   The current rotation of the robot in radians
     * @param isHoldController Whether or not the controller should use the hold controller PID tuning
     * @return The controller output
     */
    public double calculate(double goalRadians, double currentRadians, boolean isHoldController) {
        double measurement = currentRadians;
        calculatedValue = controller.calculate(measurement, goalRadians); // Based on whether or not hold controller, calculate motor power needed

        if (atSetpoint()) {
            if (isHoldController) { // If hold controller, use the hold controller's aggressive tuning
                calculatedValue = calculateHold(goalRadians, currentRadians);
                return calculatedValue;
            }
            calculatedValue = 0; // (Basically) Else: no movement necesarry for rotating controller because robot has reached goal
            return calculatedValue;
        } else {
            return calculatedValue;
        }
    }

    /**
     * Same function as {@link RotationController#calculate(double, double, boolean)},
     * but assumes the desired PID controller is the rotating controller. 
     */
    public double calculate(double goalRadians, double currentRadians) {
        return calculate(goalRadians, currentRadians, false);
    }

    /**
     * Calculates the power needed for the Hold controller to maintain the robot's rotation
     * @param goalRadians       The goal rotation of the robot in radians
     * @param currentRadians    The current rotation of the robot in radians
     * @return The hold controller's output
     */
    public double calculateHold(double goalRadians, double currentRadians) {
        calculatedValue = holdController.calculate(currentRadians, goalRadians);
        return calculatedValue;
    }

    public boolean atSetpoint() {
        return controller.atSetpoint();
    }

    /**
     * Zeroes both rotational controllers, feeding the current rotation as the zero point.
     * @param currentRadians    The current rotation of the robot in radians
     */
    public void reset(double currentRadians) {
        controller.reset(currentRadians);
        holdController.reset();
    }

    /**
     * Updates the PID values of the rotating controller; the hold controller does not change.
     * @param kP    The proportional coefficient. Must be >= 0.
     * @param kI    The integral coefficient. Must be >= 0.
     * @param kD    The differential coefficient. Must be >= 0.
     */
    public void updatePID(double kP, double kI, double kD) {
        controller.setPID(kP, kI, kD);
    }
}