package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Rotation;
import static edu.wpi.first.units.Units.Volts;
import static frc.robot.Konstants.SwerveConstants.kCANivoreNameString;
import static frc.robot.Konstants.SwerveConstants.kChassisLength;
import static frc.robot.Konstants.SwerveConstants.kChassisWidth;
import static frc.robot.Konstants.SwerveConstants.kDriveD;
import static frc.robot.Konstants.SwerveConstants.kDriveI;
import static frc.robot.Konstants.SwerveConstants.kDriveP;
import static frc.robot.Konstants.SwerveConstants.kMaxVelocityMetersPerSecond;
import static frc.robot.Konstants.SwerveConstants.kPIDControllerToleranceDegrees;
import static frc.robot.Konstants.SwerveConstants.kWheelCircumferenceMeters;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.PositionTorqueCurrentFOC;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.hardware.core.CoreCANcoder;
import com.ctre.phoenix6.swerve.utility.PhoenixPIDController;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Angle;


public class SK25SwerveModule {
    TalonFX driveMotor;
    TalonFX turnMotor;
    CoreCANcoder encoder;
    PhoenixPIDController turnPID;
    SlewRateLimiter velocityLimiter;
    Double encoderOffset;
    Double inverted;



    /* Be able to switch which control request to use based on a button press */
    /* Start at position 0, use slot 0 */
    private final PositionVoltage m_positionVoltage = new PositionVoltage(0).withSlot(0);
    /* Start at position 0, use slot 1 */
    private final PositionTorqueCurrentFOC m_positionTorque = new PositionTorqueCurrentFOC(0).withSlot(1);
    /* Keep a brake request so we can disable the motor */
    private final NeutralOut m_brake = new NeutralOut();


    public SK25SwerveModule(int driveMotorID, int turnMotorID, int encoderID, double encoderOffset, double inverted)
    {
        //the drive motor to use for this module
        driveMotor = new TalonFX(driveMotorID, kCANivoreNameString);
        //the turn motor to use for this module
        turnMotor = new TalonFX(turnMotorID, kCANivoreNameString);
        //the encoder to use for this module
        encoder = new CoreCANcoder(encoderID, kCANivoreNameString);
        //the Phoenix PID controller to use for this module
        turnPID = new PhoenixPIDController(kDriveP, kDriveI, kDriveD);
        //makes a new SlewrateLimiter to limit the velocity of the module
        velocityLimiter = new SlewRateLimiter(kMaxVelocityMetersPerSecond);
        
        //reset PID controler
        turnPID.reset();

        this.encoderOffset = encoderOffset;
        this.inverted = inverted;

        //allows the PID loop to take the smaller of the two errors, for example, traveling -90 degrees instead of 270.
        turnPID.enableContinuousInput(-180, 180);
        //sets the acceptable error bound to which the controller will stop if reached
        turnPID.setTolerance(kPIDControllerToleranceDegrees);


        TalonFXConfiguration configs = new TalonFXConfiguration();
        configs.Slot0.kP = 2.4; // An error of 1 rotation results in 2.4 V output
        configs.Slot0.kI = 0; // No output for integrated error
        configs.Slot0.kD = 0.1; // A velocity of 1 rps results in 0.1 V output
        // Peak output of 8 V
        configs.Voltage.withPeakForwardVoltage(Volts.of(8))
            .withPeakReverseVoltage(Volts.of(-8));

        configs.Slot1.kP = 60; // An error of 1 rotation results in 60 A output
        configs.Slot1.kI = 0; // No output for integrated error
        configs.Slot1.kD = 6; // A velocity of 1 rps results in 6 A output
        // Peak output of 120 A
        configs.TorqueCurrent.withPeakForwardTorqueCurrent(Amps.of(120))
            .withPeakReverseTorqueCurrent(Amps.of(-120));

        turnMotor.setPosition(0);
    }

    //the method which controls the drive and turn motors, using open loop for drive and closed for turn
    public void driveOpenLoop(SwerveModuleState desiredState)
    {
        //decrease the error of the module states using their current rotation
        decreaseError(desiredState);
        //determine the voltage output of the drive motor based on its speed in m/s
        double percentOutput = desiredState.speedMetersPerSecond; // / kMaxVelocityMetersPerSecond;
        //set the drive motor output as a percentage times the number of volts supplied by the battery (12 volts)
        driveMotor.setVoltage(inverted * percentOutput * 12);//getLimitedVelocity(percentOutput * 12));
        //set the PID controller to reach the desired angle
        //applyPID(desiredState.angle.getMeasure()); // Always closed-loop control for turn motor.

        //set the motor to the target rotation value
        turnMotor.setControl(m_positionVoltage.withPosition(desiredState.angle.getMeasure()));  //TODO: barke mode and is rotatoin2d in rotations??

        //if no output for drive or turn, set motor to brake mode:
        //driveMotor.setControl(m_brake);
        //turnMotor.setControl(m_brake);
    }

    /** gets the absolute position of the turn motor in rotations */
    private StatusSignal<Angle> getEncoderAbsRotations()
    {
        return encoder.getAbsolutePosition();
    }

    private Angle getDriveDistanceAngle()
    {
        //converts from StatusSignal<Angle> to Angle with the getValue() method.
        return driveMotor.getPosition().getValue();
    }

    private Double getDriveDistanceMeters()
    {
        return getDriveDistanceAngle().in(Rotation) * kWheelCircumferenceMeters;
    }


    //translation2d objects define movement on an xy pane. these ones are for the module's distance from the center of the robot with x and y coordinates
    Translation2d moduleTranslation = new Translation2d(kChassisWidth / 2.0, kChassisLength / 2.0);


    //gets the rotation of the swervemodule
    /**
     * Gets the rotation of the swerve module, aka the position of the turn motor.
     * @return The module's rotation.
     */
    private Rotation2d getModuleRotation()
    {
        //rotation2d is a rotation coordinate on the unit circle. This version of the method takes radian values as doubles (0.0 to 2 * Math.PI).
        //this object holds an angle with turning encoder's current pos.
        return new Rotation2d(getOffsetEncoderPos(encoder, encoderOffset));
    }

    /**
     * Gets the change in position from a starting poisition to an ending position and the rotation value in one object as a matrix. 
     * The Transform2d class uses ideas of matrix operations to apply rotation via the rotational matrix
     * It also preforms vector operations to determine the change in translation.
     * The rotatoinal matrix is as follows, where x is the degrees to rotate:
     * <p>
     * Row 1: [cosx  -sinx  0] <p>
     * Row 2: [sinx  cosx   0] <p>
     * Row 3: [0     0      1] <p>
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
    public SwerveModulePosition getLatestSwerveModulePosition()
    {
        //new swerve module position with the current distance driven and current module rotation
        return new SwerveModulePosition(getDriveDistanceMeters(), getModuleRotation());
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
        return (getEncoderAbsRotations().getValue().in(Units.Radians) - offset);
    }

     /**
      * Gets the velocity after a velocityLimiter object caps the maximum velocity.
      * @param velocity The velocity to be limited.
      * @return The limited velocity.
      */
     private double getLimitedVelocity(double velocity)
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
    private void decreaseError(SwerveModuleState state)
    {
            //ensures the wheels turn in the direction of least distance travelled (smaller angle).
            state.optimize(getModuleRotation());

            // Scale speed by cosine of angle error. This scales down movement perpendicular to the desired
            // direction of travel that can occur when modules change directions. This results in smoother
            // driving.
            state.cosineScale(getModuleRotation());
    }
 
    /**
     * Applys a PID loop to the turn motor using PID constants and the target setpoint of the PID.
     * @param setpoint The setpoint of the PID loop to reach.
     */
    private void applyPID(Angle setpoint)
    {
        final MotionMagicVoltage angleVoltageControl = new MotionMagicVoltage(0);
        angleVoltageControl.UpdateFreqHz = 0;

        //set position to a rotation distance specified by the request
        turnMotor.setControl(angleVoltageControl.withPosition(setpoint));

        //turnMotor.setPosition(turnPID.calculate(encoder.getAbsolutePosition(), setpoint, encoder.Timestamp.getTimestamp()))
    }
}
