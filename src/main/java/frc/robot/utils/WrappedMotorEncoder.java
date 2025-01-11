// package frc.robot.utils;

// import com.ctre.phoenix6.configs.TalonFXConfiguration;
// import com.ctre.phoenix6.hardware.TalonFX;

// import edu.wpi.first.math.MathUtil;
// import edu.wpi.first.math.geometry.Rotation2d;

// /**
//  * This is a wrapper class used to make the Falcon500 (TalonFX) CAN encoders provide the
//  * outputs we need to drive the swerve module to the correct position
//  */
// public class WrappedMotorEncoder
// {
//     /**
//      * The underlying motor for this MotorEncoder.
//      */
//     private TalonFX underlyingMotor;


//     /**
//      * Indicates the initial offset of the motor in degrees
//      */
//     private double degreesOffset = 0.0;

//     /**
//      * Indicates the initial offset of the motor in degrees
//      */
//     private double degreesPerPulse = 0.0;

//     /**
//      * Sets up the encoder wrapper with the required motor and other informaiton to
//      * calculate distance and velocities
//      * 
//      * @param selectedMotorEncoder
//      *            The motor used to provide the base information for this MotorEncoder
//      * @param degreesPerRotation
//      *            The ratio of sensor rotations to the mechanism's output
//      * @param degreesOffset
//      *            Indicates the initial absolute position of the motor
//      */
//     public WrappedMotorEncoder(TalonFX selectedMotorEncoder, double degreesPerRotation,
//         double degreesOffset)
//     {
//         underlyingMotor.setPosition(0);
//         underlyingMotor = selectedMotorEncoder;
//         this.degreesOffset = degreesOffset;
//         this.degreesPerPulse = degreesPerRotation;

//         TalonFXConfiguration config = new TalonFXConfiguration();
//         config.Feedback.SensorToMechanismRatio = degreesPerRotation;
//         underlyingMotor.getConfigurator().apply(config);
//     }

//     /**
//      * Calculates the position of the motor in rotations using the current position,
//      * inversion, and the last motor position reset.
//      * 
//      * @return The current position of the motor in rotations
//      */
//     private double getPositionRotations()
//     {
//         return underlyingMotor.getPosition().getValue();
//     }

//     /**
//      * Calculates the velocity of the motor in rot/sec using the inversion, and the last
//      * motor position reset.
//      * 
//      * @return The current velocity of the motor in rotations per second
//      */
//     private double getVelocityRotations()
//     {
//         return underlyingMotor.getVelocity().getValue();
//     }

//     /**
//      * Calculates the position of the motor in meters using the stored meters per rotation
//      * and the current position.
//      * 
//      * @return The current position of the motor in meters
//      */
//     public double getPositionDegrees()
//     {
//         return MathUtil.inputModulus((getPositionRotations() * 360 + degreesOffset), 0,
//             360); //Multiply by 360 to get degrees from rotations
//     }

//     /**
//      * Calculates the velocity of the motor in meters per second using the stored meters
//      * per pulse and the current velocity.
//      * 
//      * @return The current velocity of the motor in meters
//      */
//     public double getVelocityDegrees()
//     {
//         return getVelocityRotations() * 360; //Multiply by 360 to get degrees from rotations
//     }

//     public double getRotationPositionFromDesiredAngle(double angle)
//     {
//         double offset = MathUtil.inputModulus(angle - getPositionDegrees(), 0, 360);
//         double absoluteDistance = Math.abs(offset);
//         if(absoluteDistance > 90)
//         {
//             offset = (360-absoluteDistance) * -Math.signum(offset);
//         }
//         return getPositionRotations() + offset / 360;
//     }

//     public Rotation2d getRotation2d()
//     {
//         return Rotation2d.fromDegrees(getPositionDegrees());
//     }
// }
