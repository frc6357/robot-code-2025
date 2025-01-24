package frc.robot.subsystems;

import static edu.wpi.first.units.Units.MetersPerSecond;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.hardware.core.CoreCANcoder;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.AngleUnit;
import edu.wpi.first.units.Units;

public class SwerveWheel {

    //declare variables for constructor
    PIDController directionController;
    TalonFX driveMotor;
    TalonFX turnMotor;
    CoreCANcoder encoder;
    double offset;

    //declare and intitialize the encoder distance variables to zero before further manipulation.
    double currentDistance = 0.0;
    double targetDistance = 0.0;

    /**
     * Creates a new SwerveWheel which provides methods to control the drive and turn motors of the robot.
     * @param P The P value in the turning motor's PID controller.
     * @param I The I value in the turning motor's PID controller.
     * @param D The D value in the turning motor's PID controller. 
     * @param driveID The ID of the drive motor to be used with this SwerveWheel.
     * @param driveID The ID of the drive motor to be used with this SwerveWheel.
     * @param driveID The ID of the drive motor to be used with this SwerveWheel.
     * @param offset The offset constant of the encoder.
     */
    public SwerveWheel(double P, double I, double D, int driveID, int turnID, int encoderID, double offset)
    {
        //initialize constructor variables
        directionController = new PIDController(P, I, D);
        driveMotor = new TalonFX(driveID);
        turnMotor = new TalonFX(turnID);
        encoder = new CoreCANcoder(encoderID);
        this.offset = offset;
    }

    /**
     * Gets the value of the encoder after the applied offset.
     * @param encoderOffset 
     * @return Returns the encoder value after accounting for its offset.
     */
    public double getOffsetEncoderValue(double encoderOffset)
    {
        //get the Angle of the encoder as type Angle, converted from the defualt StatusSingal<Angle> using the getValue() method.
        Angle encoderAngle = encoder.getAbsolutePosition().getValue();

        //converts the Angle type to a primative double of the respective angle.
        Double encoderDouble = encoderAngle.in(Units.Degrees);

        //return the encoder double value with the offset.
        return (encoderDouble - encoderOffset);
    }

    /**
     * Sets the direction of the wheels by setting the turn motor to the calculated difference between the current and target pose of the turn motor.
     * @param setpoint The target position of the turning motor to reach as a percentage value between 0.0 and 1.0.
     */
    public void setDirection(double setpoint)
    {
        directionController.reset();
        //allows the PID loop to take the smaller of the two errors, for example, traveling -90 degrees instead of 270.
        directionController.enableContinuousInput(-180, 180);
        //sets the acceptable error bound to which the controller will stop if reached
        directionController.setTolerance(0.2);
        //sets the target value for the PID controller
        //directionController.setSetpoint(setpoint);

        //get encoder pos
        double encoderPos = getOffsetEncoderValue(offset);
        //set the motor to the target PID condition. Negative sign since all turn motors are inverted.
        turnMotor.set(-directionController.calculate(encoderPos, setpoint));
    }

    
    /**
     * Sets the speed of the drive motor.
     * @param speed The speed of the motor as a percentage between 0.0 and 1.0.
     */
    public void setSpeed(double speed)
    {
        driveMotor.set(speed);
    }
}
