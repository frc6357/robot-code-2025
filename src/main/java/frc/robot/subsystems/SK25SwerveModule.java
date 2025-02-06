package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Rotation;
import static frc.robot.Konstants.SwerveConstants.kCANivoreName;
import static frc.robot.Konstants.SwerveConstants.kChassisLength;
import static frc.robot.Konstants.SwerveConstants.kChassisWidth;
import static frc.robot.Konstants.SwerveConstants.kDriveD;
import static frc.robot.Konstants.SwerveConstants.kDriveI;
import static frc.robot.Konstants.SwerveConstants.kDriveP;
import static frc.robot.Konstants.SwerveConstants.kMaxVelocity;
import static frc.robot.Konstants.SwerveConstants.kPIDControllerTolerance;
import static frc.robot.Konstants.SwerveConstants.kWheelCircumference;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.hardware.core.CoreCANcoder;
import com.ctre.phoenix6.swerve.utility.PhoenixPIDController;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;



public class SK25SwerveModule {
    TalonFX driveMotor;
    TalonFX turnMotor;
    CoreCANcoder encoder;
    PhoenixPIDController turnPID;
    SlewRateLimiter velocityLimiter;
    Double encoderOffset;

    public SK25SwerveModule(int driveMotorID, int turnMotorID, int encoderID, double encoderOffset)
    {
        //the drive motor to use for this module
        driveMotor = new TalonFX(driveMotorID, kCANivoreName);
        //the turn motor to use for this module
        turnMotor = new TalonFX(turnMotorID, kCANivoreName);
        //the encoder to use for this module
        encoder = new CoreCANcoder(encoderID, kCANivoreName);
        //the Phoenix PID controller to use for this module
        turnPID = new PhoenixPIDController(kDriveP, kDriveI, kDriveD);
        //makes a new SlewrateLimiter to limit the velocity of the module
        velocityLimiter = new SlewRateLimiter(kMaxVelocity);
        
        //reset PID controler
        turnPID.reset();

        this.encoderOffset = encoderOffset;
    }

    //gets the velocity of the turn motor
    StatusSignal<AngularVelocity> fLTurnVelocity = encoder.getVelocity();
    //gets the absolute position of the turn motor
    StatusSignal<Angle> fLTurnDistance = encoder.getAbsolutePosition();
    //gets the velocity of the drive motor
    double fLDriveVelocity = driveMotor.get();
    //converts from StatusSignal<Angle> to Angle with the getValue() method.
    Angle fLDriveDistance = driveMotor.getPosition().getValue();

    //degrees of the drive motor
    Double driveRotations = fLDriveDistance.in(Units.Degrees);
    //meters of distance travelled by the wheel
    Double driveDistance = fLDriveDistance.in(Rotation) * kWheelCircumference;


    //rotation2d is a rotation coordinate on the unit circle. This version of the method takes radian values as doubles (0.0 to 2 * Math.PI).
    //this object holds an angle with turning encoder's current pos.
    Rotation2d moduleRotation = new Rotation2d(getOffsetEncoderPos(encoder, encoderOffset));

    //translation2d objects define movement on an xy pane. these ones are for the module's distance from the center of the robot with x and y coordinates
    Translation2d moduleTranslation = new Translation2d(kChassisWidth / 2.0, kChassisLength / 2.0);

    //represents the distance travelled in an arc, dx is distance traveled in a vector (translation), dtheta is angle of travel, 
    //and dy is the distance driven to the side (0.0 since swerve dosnt do this).
    Twist2d acrDistanceTraveled = new Twist2d(5.0, 0.0, 45.0);


    //gets the rotation of the swervemodule
    /**
     * Gets the rotation of the swerve module, aka the position of the turn motor.
     * @return The module's rotation.
     */
    public Rotation2d getModuleRotation()
    {
        return moduleRotation;
    }

    /**
     * Gets the translation of the module: the distance between the module and the center of the robot.
     * @return the Translation of the module form the center of the robot.
     */
    //public Translation2d getModuleTranslation()
    //{
        //return moduleTranslation;
    //}

    /**
     * Gets the change in position from a starting poisition to an ending position and the rotation value in one object as a matrix. 
     * The Transform2d class uses ideas of matrix operations to apply rotation via the rotational matrix
     * It also preforms vector operations to determine the change in translation.
     * The rotatoinal matrix is as follows, where x is the degrees to rotate:
     * 
     * Row 1: [cosx  -sinx  0]
     * Row 2: [sinx  cosx   0]
     * Row 3: [0     0      1]
     * 
     * @param startPose The starting position of the robot before the transformation occurs.
     * @param endPose The ending position of the robot after the transformation occurs.
     * @return The transformation from the startPose to endPose.
     */
    public Transform2d getTransformation(Pose2d startPose, Pose2d endPose)
    {
        return new Transform2d(startPose, endPose);
    }
   
    //gets the translation2d of the transform2d object
    /**
     * Gets the underlying translation of the occured transformation.
     * @param transformation The transformation preformed.
     * @return The translation preformed by the Transform2d object.
     */
    public Translation2d getTranslation2d(Transform2d transformation)
    {
        //returns the translation
        return transformation.getTranslation();
    }
   
    /**
     * Gets the underlying rotation of the occured transformation.
     * @param transformation The transformation occured.
     * @return The rotation preformed by the Transform2d object.
     */
    public Rotation2d getRotation2d(Transform2d transformation)
    {
        //returns the rotation
        return transformation.getRotation();
    }

    //gets the swerve module position
    /**
     * Gets the position of the swerve modules on the feild using the module rotation and distance driven by the drive motor.
     * @return The object containing the module position.
     */
    public SwerveModulePosition getSwerveModulePosition()
    {
        //new swerve module position with the current distance driven and current module rotation
        return new SwerveModulePosition(driveDistance, moduleRotation);
    }
    
    /**
     * Gets the output of the encoder after accounting for its offsets.
     * @param encoder The encoder to use.
     * @param offset The offset of the encoder in radians.
     * @return The encoder position in radians with respect to the specified offset.
     */
    private double getOffsetEncoderPos(CoreCANcoder encoder, Double offset)
    {
        //convert the StatusSignal<Angle> return type of the encoder to an angle, then to radians, then substract the offset.
        return (encoder.getAbsolutePosition().getValue().in(Units.Radians) - offset);
    }

     /**
      * Gets the velocity after a velocityLimiter object caps the maximum velocity.
      * @param velocity The velocity to be limited.
      * @return The limited velocity.
      */
     public double getLimitedVelocity(double velocity)
     {
         //returns the velocity after the velocity limit has been applied. Pass the velocity to be limited in the calculate method.
         return velocityLimiter.calculate(velocity);
     }

    /**
     * Optimizes the module states through wheel pathing reduction and cosine scaling.
     * The state is told to spin the wheels in the direction which travels a shorter distance.
     * For example, if the wheel is at 0 degrees and has a target of 270 degrees, the wheel can rotate 
     * -90 degrees instead of 270 to reach its destination more efficiently.
     * The state also scales down any perpendicular movement to the traget direction to smoothen the driving.
     * It takes the error of the angle finds the cosine of the angle to scale the speed with.
     * @param state The swerve module state to be optimized.
     * @param gyroAngle the current rotation of the robot, obtained from the gyro.
     */
    public void decreaseError(SwerveModuleState state, Rotation2d gyroAngle)
    {
            //ensures the wheels turn in the direction of least distance travelled (smaller angle).
            state.optimize(gyroAngle);

            // Scale speed by cosine of angle error. This scales down movement perpendicular to the desired
            // direction of travel that can occur when modules change directions. This results in smoother
            // driving.
            state.cosineScale(gyroAngle);
    }

    /**
     * Applys a PID loop to the turn motor using PID constants and the target setpoint of the PID.
     * @param setpoint The setpoint of the PID loop to reach.
     */
    private void applyPID(Angle setpoint)
    {
        //allows the PID loop to take the smaller of the two errors, for example, traveling -90 degrees instead of 270.
        turnPID.enableContinuousInput(-180, 180);
        //sets the acceptable error bound to which the controller will stop if reached
        turnPID.setTolerance(kPIDControllerTolerance);
        //create P, I, and D configs for the PID Configs object
        Slot0Configs turnPIDConfigs = new Slot0Configs();
        turnPIDConfigs.kP = kDriveP;
        turnPIDConfigs.kI = kDriveI;
        turnPIDConfigs.kD = kDriveD; 
        //apply the PID Configs to the motor
        turnMotor.getConfigurator().apply(turnPIDConfigs);
        //create a position closed-loop request, voltage output, slot 0 configs. This defines the setpoint. 
        final PositionVoltage m_request = new PositionVoltage(setpoint).withSlot(0);
        //set position to a rotation distance specified by the request
        turnMotor.setControl(m_request);
    }

    /**
     * Sets the target state of the swerve modules by applying the PID loop 
     * to the turn motor and setting the drive motor to the specified velocity.
     * @param setpoint The setpoint of the applied PID loop to reach.
     * @param velocity The velocity to spin the drive motor at, at a percentage from 0.0 to 1.0.
     */
    public void setTargetState(Angle setpoint, double velocity)
    {
        //apply the PID constants to the turn motor
        applyPID(setpoint);
        //set the drive motor to the velocity
        driveMotor.set(velocity);
    }
}
