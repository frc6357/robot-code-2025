package frc.robot.subsystems.swerve;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.Volts;
import static frc.robot.Konstants.SwerveConstants.kBackLeftEncoderOffsetRadians;
import static frc.robot.Konstants.SwerveConstants.kBackLeftXPos;
import static frc.robot.Konstants.SwerveConstants.kBackLeftYPos;
import static frc.robot.Konstants.SwerveConstants.kBackRightEncoderOffsetRadians;
import static frc.robot.Konstants.SwerveConstants.kBackRightXPos;
import static frc.robot.Konstants.SwerveConstants.kBackRightYPos;
import static frc.robot.Konstants.SwerveConstants.kCANivoreNameString;
import static frc.robot.Konstants.SwerveConstants.kCoupleGearRatio;
import static frc.robot.Konstants.SwerveConstants.kDriveClosedLoopOutput;
import static frc.robot.Konstants.SwerveConstants.kDriveFrictionVoltage;
import static frc.robot.Konstants.SwerveConstants.kDriveGains;
import static frc.robot.Konstants.SwerveConstants.kDriveGearRatio;
import static frc.robot.Konstants.SwerveConstants.kDriveInertia;
import static frc.robot.Konstants.SwerveConstants.kFrontLeftEncoderOffsetRadians;
import static frc.robot.Konstants.SwerveConstants.kFrontLeftXPos;
import static frc.robot.Konstants.SwerveConstants.kFrontLeftYPos;
import static frc.robot.Konstants.SwerveConstants.kFrontRightEncoderOffsetRadians;
import static frc.robot.Konstants.SwerveConstants.kFrontRightXPos;
import static frc.robot.Konstants.SwerveConstants.kFrontRightYPos;
import static frc.robot.Konstants.SwerveConstants.kInvertLeftSide;
import static frc.robot.Konstants.SwerveConstants.kInvertRightSide;
import static frc.robot.Konstants.SwerveConstants.kIsBackLeftEncoderInverted;
import static frc.robot.Konstants.SwerveConstants.kIsBackRightEncoderInverted;
import static frc.robot.Konstants.SwerveConstants.kIsFrontLeftEncoderInverted;
import static frc.robot.Konstants.SwerveConstants.kIsFrontRightEncoderInverted;
import static frc.robot.Konstants.SwerveConstants.kSlipCurrentAmps;
import static frc.robot.Konstants.SwerveConstants.kSpeedAt12VoltsMeterPerSecond;
import static frc.robot.Konstants.SwerveConstants.kTurnClosedLoopOutput;
import static frc.robot.Konstants.SwerveConstants.kTurnFrictionVoltage;
import static frc.robot.Konstants.SwerveConstants.kSteerGains;
import static frc.robot.Konstants.SwerveConstants.kTurnGearRatio;
import static frc.robot.Konstants.SwerveConstants.kSteerInertia;
import static frc.robot.Konstants.SwerveConstants.kTurnMotorsReversed;
import static frc.robot.Konstants.SwerveConstants.kTurningCurrentLimitAmps;
import static frc.robot.Konstants.SwerveConstants.kTurningCurrentLimitsEnabled;
import static frc.robot.Konstants.SwerveConstants.kWheelRadiusInches;
import static frc.robot.Ports.DrivePorts.kBackLeftDriveMotorPort;
import static frc.robot.Ports.DrivePorts.kBackLeftEncoderPort;
import static frc.robot.Ports.DrivePorts.kBackLeftTurnMotorPort;
import static frc.robot.Ports.DrivePorts.kBackRightDriveMotorPort;
import static frc.robot.Ports.DrivePorts.kBackRightEncoderPort;
import static frc.robot.Ports.DrivePorts.kBackRightTurnMotorPort;
import static frc.robot.Ports.DrivePorts.kFrontLeftDriveMotorPort;
import static frc.robot.Ports.DrivePorts.kFrontLeftEncoderPort;
import static frc.robot.Ports.DrivePorts.kFrontLeftTurnMotorPort;
import static frc.robot.Ports.DrivePorts.kFrontRightDriveMotorPort;
import static frc.robot.Ports.DrivePorts.kFrontRightEncoderPort;
import static frc.robot.Ports.DrivePorts.kFrontRightTurnMotorPort;
import static frc.robot.Ports.DrivePorts.kPigeonPort;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.Pigeon2Configuration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.swerve.SwerveDrivetrainConstants;
import com.ctre.phoenix6.swerve.SwerveModuleConstants;
import com.ctre.phoenix6.swerve.SwerveModuleConstants.SteerFeedbackType;
import com.ctre.phoenix6.swerve.SwerveModuleConstantsFactory;

import edu.wpi.first.units.measure.Angle;
import lombok.Getter;
import lombok.Setter;

//"Straight outta Spectrum"

public class SwerveConstantsConfigurator {

    //Angular heading used for robot rotation control.
    //Heading is the rotation of the robot from its default position.
    @Getter @Setter private double targetHeading = 0;

    // Initial configs for the drive and steer motors and the CANcoder; these cannot be null.
    // Some configs will be overwritten; check the `with*InitialConfigs()` API documentation.
    @Getter private TalonFXConfiguration driveInitialConfigs = new TalonFXConfiguration();

    /** Create current limits for the turning motors, since they don't need significant current or voltage to
     * their respective modules.
     */
    @Getter private TalonFXConfiguration steerInitialConfigs =
            new TalonFXConfiguration().withCurrentLimits(new CurrentLimitsConfigs()
                // Swerve azimuth does not require much torque output, so we can set a relatively low 
                //stator current limit to help avoid brownouts without impacting performance.
                .withStatorCurrentLimit(kTurningCurrentLimitAmps)
                .withStatorCurrentLimitEnable(kTurningCurrentLimitsEnabled));

                
    @Getter private CANcoderConfiguration cancoderInitialConfigs = new CANcoderConfiguration();
    // Configs for the Pigeon 2; leave this null to skip applying Pigeon 2 configs
    // TODO: Check Pigeon usage in Swerve
    @Getter private Pigeon2Configuration pigeonConfigs = null;

    
    //TODO: see if these encoder offset variables are needed for the update method at the bottom of SwerveConfig
    @Getter private Angle frontLeftEncoderOffset = Rotations.of(kFrontLeftEncoderOffsetRadians);
    @Getter private Angle frontRightEncoderOffset = Rotations.of(kFrontRightEncoderOffsetRadians);
    @Getter private Angle backLeftEncoderOffset = Rotations.of(kBackLeftEncoderOffsetRadians);
    @Getter private Angle backRightEncoderOffset = Rotations.of(kBackRightEncoderOffsetRadians);


    /* Contains a set of constants for a swerve drivetrain */
    @Getter private SwerveDrivetrainConstants drivetrainConstants;

    /**Constants for each swerve module (stored in SwerveModuleConstants objects)
    * which are also constant across all four modules are dumped into
    *the SwerveModuleConstantsFactory object.*/
    @SuppressWarnings("rawtypes")
    @Getter private SwerveModuleConstantsFactory constantCreator;

    //Create new SwerveModuleConstants objects for each module.
    //These will hold constants values for each module.
    @SuppressWarnings("rawtypes") @Getter private SwerveModuleConstants frontLeft;
    @SuppressWarnings("rawtypes") @Getter private SwerveModuleConstants frontRight;
    @SuppressWarnings("rawtypes") @Getter private SwerveModuleConstants backLeft;
    @SuppressWarnings("rawtypes") @Getter private SwerveModuleConstants backRight;

    //Makes an array of the swerve module constants objects
    @SuppressWarnings("rawtypes")
    public SwerveModuleConstants[] getModules() {
        return new SwerveModuleConstants[] {frontLeft, frontRight, backLeft, backRight};
    }


    /** Creates a new SwerveConstantsConfigurator which applies all constants to the drivetrain and its modules.*/
    public SwerveConstantsConfigurator() {
        updateConfig();
    }

    /** Applies the constant values (configs) to the SwerveModuleConstants objects for each module.
     * Also applies constant values to the SwerveDrivetrainConstants object and the 
     * SwerveModuleConstantsFactory object.
     * @return the SwerveConfig object used to call this method.
    */
    @SuppressWarnings({ "rawtypes", "unchecked" }) 
    public SwerveConstantsConfigurator updateConfig() {
        //apply drivetrain constants
        drivetrainConstants =
                new SwerveDrivetrainConstants()
                        .withCANBusName(kCANivoreNameString)
                        .withPigeon2Id(kPigeonPort.ID)
                        .withPigeon2Configs(pigeonConfigs);

        //apply module factory constants
        constantCreator =
                new SwerveModuleConstantsFactory()
                        .withDriveMotorGearRatio(kDriveGearRatio)
                        .withSteerMotorGearRatio(kTurnGearRatio)
                        .withWheelRadius(Inches.of(kWheelRadiusInches))
                        .withSlipCurrent(Amps.of(kSlipCurrentAmps))
                        .withSteerMotorGains(kSteerGains)
                        .withDriveMotorGains(kDriveGains)
                        .withSteerMotorClosedLoopOutput(kTurnClosedLoopOutput)
                        .withDriveMotorClosedLoopOutput(kDriveClosedLoopOutput)
                        .withSpeedAt12Volts(kSpeedAt12VoltsMeterPerSecond)
                        .withSteerInertia(kSteerInertia)
                        .withDriveInertia(kDriveInertia)
                        .withSteerFrictionVoltage(Volts.of(kTurnFrictionVoltage))
                        .withDriveFrictionVoltage(Volts.of(kDriveFrictionVoltage))
                        .withFeedbackSource(SteerFeedbackType.FusedCANcoder)  //TODO: change feeback to something without phoenix pro
                        .withCouplingGearRatio(kCoupleGearRatio)
                        .withDriveMotorInitialConfigs(driveInitialConfigs)
                        .withSteerMotorInitialConfigs(steerInitialConfigs);

        //apply front left module constants
        frontLeft =
                constantCreator.createModuleConstants(
                        kFrontLeftTurnMotorPort.ID,
                        kFrontLeftDriveMotorPort.ID,
                        kFrontLeftEncoderPort.ID,
                        Radians.of(kFrontLeftEncoderOffsetRadians),
                        Inches.of(kFrontLeftXPos),
                        Inches.of(kFrontLeftYPos),
                        kInvertLeftSide,
                        kTurnMotorsReversed,
                        kIsFrontLeftEncoderInverted);

        //apply front right module constants
        frontRight =
                constantCreator.createModuleConstants(
                        kFrontRightTurnMotorPort.ID,
                        kFrontRightDriveMotorPort.ID,
                        kFrontRightEncoderPort.ID,
                        Radians.of(kFrontRightEncoderOffsetRadians),
                        Inches.of(kFrontRightXPos),
                        Inches.of(kFrontRightYPos),
                        kInvertRightSide,
                        kTurnMotorsReversed,
                        kIsFrontRightEncoderInverted);

        //apply back left module constants
        backLeft =
                constantCreator.createModuleConstants(
                        kBackLeftTurnMotorPort.ID,
                        kBackLeftDriveMotorPort.ID,
                        kBackLeftEncoderPort.ID,
                        Radians.of(kBackLeftEncoderOffsetRadians),
                        Inches.of(kBackLeftXPos),
                        Inches.of(kBackLeftYPos),
                        kInvertLeftSide,
                        kTurnMotorsReversed,
                        kIsBackLeftEncoderInverted);

        //apply back right module constants
        backRight =
                constantCreator.createModuleConstants(
                        kBackRightTurnMotorPort.ID,
                        kBackRightDriveMotorPort.ID,
                        kBackRightEncoderPort.ID,
                        Radians.of(kBackRightEncoderOffsetRadians),
                        Inches.of(kBackRightXPos),
                        Inches.of(kBackRightYPos),
                        kInvertRightSide,
                        kTurnMotorsReversed,
                        kIsBackRightEncoderInverted);

        return this;
    }

    /** Changes the encoder offset from its default value to the ones specified in this method call.
     * @param frontLeft The offset of the front left encoder in radians.
     * @param frontRight The offset of the front right encoder in radians.
     * @param backLeft The offset of the back left encoder in radians.
     * @param backRight The offset of the back right encoder in radians.
     * @return The SwerveConfig object used to call this method.
    */
    public SwerveConstantsConfigurator configEncoderOffsets(                     //TODO: find if this encoder offsets are updated from default values, then edit this method
            double frontLeft, double frontRight, double backLeft, double backRight) {
        frontLeftEncoderOffset = Rotations.of(frontLeft);
        frontRightEncoderOffset = Rotations.of(frontRight);
        backLeftEncoderOffset = Rotations.of(backLeft);
        backRightEncoderOffset = Rotations.of(backRight);
        return updateConfig();
    }
}
