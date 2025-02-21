// package frc.robot.subsystems;

// import static edu.wpi.first.units.Units.Rotation;
// import static frc.robot.Konstants.SwerveConstants.kCANivoreNameString;
// import static frc.robot.Konstants.SwerveConstants.kDriveA;
// import static frc.robot.Konstants.SwerveConstants.kDriveD;
// import static frc.robot.Konstants.SwerveConstants.kDriveI;
// import static frc.robot.Konstants.SwerveConstants.kDriveP;
// import static frc.robot.Konstants.SwerveConstants.kDriveS;
// import static frc.robot.Konstants.SwerveConstants.kDriveV;
// import static frc.robot.Konstants.SwerveConstants.kMaxAngularAcceleration;
// import static frc.robot.Konstants.SwerveConstants.kMaxAngularVelocity;
// import static frc.robot.Konstants.SwerveConstants.kMaxVelocityMetersPerSecond;
// import static frc.robot.Konstants.SwerveConstants.kRotationToleranceRadians;
// import static frc.robot.Konstants.SwerveConstants.kWheelCircumferenceMeters;

// import com.ctre.phoenix6.StatusSignal;
// import com.ctre.phoenix6.hardware.CANcoder;
// import com.ctre.phoenix6.hardware.TalonFX;
// import com.ctre.phoenix6.hardware.core.CoreCANcoder;
// import com.ctre.phoenix6.swerve.SwerveModule;
// import com.ctre.phoenix6.swerve.SwerveModuleConstants;

// import edu.wpi.first.math.filter.SlewRateLimiter;
// import edu.wpi.first.math.geometry.Rotation2d;
// import edu.wpi.first.math.geometry.Translation2d;
// import edu.wpi.first.math.kinematics.SwerveModulePosition;
// import edu.wpi.first.math.kinematics.SwerveModuleState;
// import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
// import edu.wpi.first.units.Units;
// import edu.wpi.first.units.measure.Angle;
// import frc.robot.utils.SKPIDController;

// public class TempModule extends SwerveModule<TalonFX, TalonFX, CANcoder>
// {

//     //the contsantsConfig class will handle logic with the motors, encoders, and most wpilib classes
//     //public static double moduleIndex =  this.m_moduleIdx;

//     public TalonFX driveMotor;
//     public TalonFX turnMotor;
//     public CANcoder encoder;
//     public double inverted;
//     double encoderOffset;

//     public Constraints trapezoidPIDRestraints;
//     public Translation2d moduleLocation;
//     public SlewRateLimiter velocityLimiter;
//     public SKPIDController modulePID;

//     //temporary
//     public double currentPoint = 0.0;
//     public double targetPoint = 0.0;
//     public double currentVelocity = 0.0;
//     public double targetVelocity = 0.0;
//     public double targetAcceleration = 0.0;

    

//     //constructor (configs)
//     public TempModule(
//         SwerveModuleConstants moduleConstants, 
//         double driveMotorInverted,
//         double encoderOffset, 
//         int drivetrainID, 
//         int moduleIndex
//         )
//     {  
//         //Uses lambdas to create new motors and encoder because the internal logic of the phoenix 
//         //SwerveModule class handle ID and CANbus assignments via the passed SwerveModuleConstants object.
//         super(TalonFX::new, TalonFX::new, CANcoder::new, moduleConstants, kCANivoreNameString, drivetrainID, moduleIndex);

//         trapezoidPIDRestraints = new Constraints(kMaxAngularVelocity, kMaxAngularAcceleration);
//         velocityLimiter = new SlewRateLimiter(kMaxVelocityMetersPerSecond);
//         modulePID = new SKPIDController(kDriveP, kDriveI, kDriveD, trapezoidPIDRestraints);
   
//         this.inverted = driveMotorInverted;
//         this.encoderOffset = encoderOffset;
//         this.driveMotor = this.getDriveMotor();
//         this.turnMotor = this.getSteerMotor();
//         this.encoder = this.getEncoder();
//     }


//     //the method which controls the drive and turn motors, using open loop for drive and closed for turn
//     public void driveOpenLoop(SwerveModuleState desiredState)
//     {
//         //decrease the error of the module states using their current rotation
//         decreaseError(desiredState);
//         //determine the voltage output of the drive motor based on its speed in m/s
//         double percentOutput = desiredState.speedMetersPerSecond; // / kMaxVelocityMetersPerSecond;
//         //set the drive motor output as a percentage times the number of volts supplied by the battery (12 volts)
//         driveMotor.setVoltage(inverted * getLimitedVelocity(percentOutput * 12));

//         //set the PID controller to reach the desired angle (always closed loop control)
//         // turnMotor.setPosition(
//         //     modulePID.applyAllPID(
//         //         true, 
//         //         currentPoint, 
//         //         targetPoint, 
//         //         kRotationToleranceRadians, 
//         //         kDriveS, 
//         //         kDriveV, 
//         //         kDriveA, 
//         //         currentVelocity, 
//         //         targetVelocity, 
//         //         targetAcceleration));
//     }

//     /**
//      * Optimizes the module states through wheel pathing reduction and cosine scaling.
//      * The state is told to spin the wheels in the direction which travels a shorter distance.
//      * For example, if the wheel is at 0 degrees and has a target of 270 degrees, the wheel can rotate 
//      * -90 degrees instead of 270 to reach its destination more efficiently.
//      * The state also scales down any perpendicular movement to the traget direction to smoothen the driving.
//      * It takes the error of the angle finds the cosine of the angle to scale the speed with.
//      * @param state The swerve module state to be optimized.
//      * @param gyroAngle the current rotation of the robot, obtained from the gyro.
//      */
//     private void decreaseError(SwerveModuleState state)
//     {
//             //ensures the wheels turn in the direction of least distance travelled (smaller angle).
//             state.optimize(getModuleRotation());

//             // Scale speed by cosine of angle error. This scales down movement perpendicular to the desired
//             // direction of travel that can occur when modules change directions. This results in smoother
//             // driving.
//             state.cosineScale(getModuleRotation());
//     }

//     /**
//       * Gets the velocity after a velocityLimiter object caps the maximum velocity.
//       * @param velocity The velocity to be limited.
//       * @return The limited velocity.
//       */
//       private double getLimitedVelocity(double velocity)
//       {
//           //returns the velocity after the velocity limit has been applied. Pass the velocity to be limited in the calculate method.
//           return velocityLimiter.calculate(velocity);
//       }

//      private Angle getDriveDistanceAngle()
//     {
//         //converts from StatusSignal<Angle> to Angle with the getValue() method.
//         return driveMotor.getPosition().getValue();
//     }

//     private Double getDriveDistanceMeters()
//     {
//         return getDriveDistanceAngle().in(Rotation) * kWheelCircumferenceMeters;
//     }

//     /**
//      * Gets the position of the swerve modules on the feild using the module rotation and distance driven by the drive motor.
//      * @return The object containing the module position.
//      */
//     public SwerveModulePosition getLatestSwerveModulePosition()
//     {
//         //new swerve module position with the current distance driven and current module rotation
//         return new SwerveModulePosition(getDriveDistanceMeters(), getModuleRotation());
//     }

//     //gets the rotation of the swervemodule
//     /**
//      * Gets the rotation of the swerve module, aka the position of the turn motor.
//      * @return The module's rotation.
//      */
//     private Rotation2d getModuleRotation()
//     {
//         //rotation2d is a rotation coordinate on the unit circle. This version of the method takes radian values as doubles (0.0 to 2 * Math.PI).
//         //this object holds an angle with turning encoder's current pos.
//         return new Rotation2d(getOffsetEncoderPos(encoder, encoderOffset));
//     }

//     /**
//      * Gets the output of the encoder after accounting for its offsets.
//      * @param encoder The encoder to use.
//      * @param offset The offset of the encoder in radians.
//      * @return The encoder position in radians with respect to the specified offset.
//      */
//     private double getOffsetEncoderPos(CoreCANcoder encoder, double offset)
//     {
//         //convert the StatusSignal<Angle> return type of the encoder to an angle, then to radians, then substract the offset.
//         return (getEncoderAbsRotations().getValue().in(Units.Radians) - offset);
//     }

//      /** gets the absolute position of the turn motor in rotations */
//     private StatusSignal<Angle> getEncoderAbsRotations()
//     {
//         return encoder.getAbsolutePosition();
//     }

//     public SwerveModuleState getTargetModuleState()
//     {
//         return this.getTargetState();
//     }

//     //public Transform2d m = new Transform2d(new Translation2d(), new Rotation2d());
// }
