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
    CoreCANcoder m_FLEncoder;
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
        m_FLEncoder = new CoreCANcoder(encoderID, kCANivoreName);
        //the Phoenix PID controller to use for this module
        turnPID = new PhoenixPIDController(kDriveP, kDriveI, kDriveD);
        //makes a new SlewrateLimiter to limit the velocity of the module
        velocityLimiter = new SlewRateLimiter(kMaxVelocity);
        //reset PID controler
        turnPID.reset();

        this.encoderOffset = encoderOffset;
    }

    StatusSignal<AngularVelocity> fLTurnVelocity = m_FLEncoder.getVelocity();
    StatusSignal<Angle> fLTurnDistance = m_FLEncoder.getAbsolutePosition();
    double fLDriveVelocity = driveMotor.get();
    //converts from StatusSignal<Angle> to Angle with the getValue() method.
    Angle fLDriveDistance = driveMotor.getPosition().getValue();
    //degrees of the drive motor
    Double driveRotations = fLDriveDistance.in(Units.Degrees);
    //TODO: make get methods

    
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

    //rotation2d is a rotation coordinate on the unit circle. This version of the method takes radian values as doubles (0.0 to 2 * Math.PI).
    //this object holds an angle with turning encoder's current pos.
    Rotation2d moduleRotation = new Rotation2d(getOffsetEncoderPos(m_FLEncoder, encoderOffset));

    //gets the rotation of the swervemodule
    public Rotation2d getModuleRotation()
    {
        return moduleRotation;
    }

    //translation2d objects define movement on an xy pane. these ones are for the module's distance from the center of the robot with x and y coordinates
    Translation2d moduleTranslation = new Translation2d(kChassisWidth / 2.0, kChassisLength / 2.0);

    //gets the translation of the module as the distance between the module and the center of the robot.
    public Translation2d getModuleTranslation()
    {
        return moduleTranslation;
    }


    //more accureatley and efficiently turns the wheels to the porper state
    public void decreaseError(SwerveModuleState state, Rotation2d gyroAngle)
    {
            //ensures the wheels turn in the direction of least distance travelled (smaller angle).
            state.optimize(gyroAngle);

            // Scale speed by cosine of angle error. This scales down movement perpendicular to the desired
            // direction of travel that can occur when modules change directions. This results in smoother
            // driving.
            state.cosineScale(gyroAngle);
    }

    //meters of distance travelled by the wheel
    Double driveDistance = fLDriveDistance.in(Rotation) * kWheelCircumference;
    //gets the swerve module position
    public SwerveModulePosition getSwerveModulePosition()
    {
        //new swerve module position with the current distance driven and current module rotation
        return new SwerveModulePosition(driveDistance, moduleRotation);
    }
    

    //limits the velocity: returns the limited velocity.
    public double getLimitedVelocity(double velocity)
    {
        //returns the velocity after the velocity limit has been applied. Pass the velocity to be limited in the calculate method.
        return velocityLimiter.calculate(velocity);
    }

    //transform objects turns pose2d objects into the new position of the robot as a pose2d. Transform is the transformation matrix from the old position.
    //get change in position
    public Transform2d getTransformation(Pose2d startPose, Pose2d endPose)
    {
        return new Transform2d(startPose, endPose);
    }
    //gets the translation2d of the transform2d object
    public Translation2d getTranslation2d(Transform2d transformation)
    {
        return transformation.getTranslation();
    }
    //gets the rotation of the transform2d object
    public Rotation2d getRotation2d(Transform2d transformation)
    {
        return transformation.getRotation();
    }

    //represents the distance travelled in an arc, dx is distance traveled in a vector (translation), dtheta is angle of travel, 
    //and dy is the distance driven to the side (0.0 since swerve dosnt do this).
    Twist2d acrDistanceTraveled = new Twist2d(5.0, 0.0, 45.0);
    


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
        // create a position closed-loop request, voltage output, slot 0 configs
        final PositionVoltage m_request = new PositionVoltage(setpoint).withSlot(0);
        // set position to rotation distance specified by the request
        turnMotor.setControl(m_request);
    }

    //sets the target state of the swerve modules by applying the PID configs/setpoint to the turn motor and setting the drive motor to the inputted velocity
    public void setTargetState(Angle setpoint, double velocity)
    {
        //apply the PID constants to the turn motor
        applyPID(setpoint);
        driveMotor.set(velocity);
    }
}

