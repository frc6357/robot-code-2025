package frc.robot;


import static edu.wpi.first.units.Units.*;
import static frc.robot.Konstants.PracticeSwerveConstants.*;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.Pigeon2Configuration;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.swerve.SwerveDrivetrainConstants;
import com.ctre.phoenix6.swerve.SwerveModuleConstants;
//import com.ctre.phoenix6.swerve.SwerveModuleConstants.ClosedLoopOutputType;
import com.ctre.phoenix6.swerve.SwerveModuleConstants.SteerFeedbackType;
import com.ctre.phoenix6.swerve.SwerveModuleConstantsFactory;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.*;
import frc.robot.Konstants.PracticeSwerveConstants;

//"Straight outta Spectrum"

public class SwerveConfig {
    private static final double kSimLoopPeriod = 0.005;
    private double robotWidth = Units.inchesToMeters(kChassisWidth);
    private double robotLength = Units.inchesToMeters(kChassisLength);

    private double deadband = kDeadband;

    // Rotation Controller Constants
    private double maxAngularVelocity = 2 * Math.PI; // rad/s
    private double maxAngularAcceleration = Math.pow(maxAngularVelocity, 2); // rad/s^2
    private double kPRotationController = 8.0;
    private double kIRotationController = 0.0;
    private double kDRotationController = 0.2;
    private double rotationTolerance = (Math.PI / 360); // rads

    private double kPHoldController = 12.0;
    private double kIHoldController = 0.0;
    private double kDHoldController = 0.0;


    private final Rotation2d BlueAlliancePerspectiveRotation = Rotation2d.fromDegrees(0);
    private final Rotation2d RedAlliancePerspectiveRotation = Rotation2d.fromDegrees(180);

    //TODO: Update PracticeSwerveConstants reference in final robot code
    // The steer motor uses any SwerveModule.SteerRequestType control request with the
    // output type specified by SwerveModuleConstants.SteerMotorClosedLoopOutput
    private Slot0Configs steerGains = PracticeSwerveConstants.steerGains;

    // When using closed-loop control, the drive motor uses the control
    // output type specified by SwerveModuleConstants.DriveMotorClosedLoopOutput
    private Slot0Configs driveGains = PracticeSwerveConstants.driveGains;

    // The stator current at which the wheels start to slip;
    // This needs to be tuned to your individual robot
    private Current slipCurrent = Amps.of(kSlipCurrentA);

    // Initial configs for the drive and steer motors and the CANcoder; these cannot be null.
    // Some configs will be overwritten; check the `with*InitialConfigs()` API documentation.
    private TalonFXConfiguration driveInitialConfigs = new TalonFXConfiguration();


    private TalonFXConfiguration steerInitialConfigs =
            new TalonFXConfiguration()
                    .withCurrentLimits(
                            new CurrentLimitsConfigs()
                                    // Swerve azimuth does not require much torque output, so we can
                                    // set a relatively low stator current limit to help avoid
                                    // brownouts without
                                    // impacting performance.
                                    .withStatorCurrentLimit(Amps.of(60))
                                    .withStatorCurrentLimitEnable(true));

    private CANcoderConfiguration cancoderInitialConfigs = new CANcoderConfiguration();
    // Configs for the Pigeon 2; leave this null to skip applying Pigeon 2 configs
    private Pigeon2Configuration pigeonConfigs = new Pigeon2Configuration();

    private LinearVelocity speedAt12Volts = MetersPerSecond.of(kSpeedAt12VoltsMps);

    private double coupleRatio = kCoupleRatio;

    private double driveGearRatio = kDriveGearRatio;
    private double steerGearRatio =kSteerGearRatio;

    private Distance wheelRadius = Inches.of(kWheelRadius);

    private boolean steerMotorReversed = kSteerMotorReversed;
    private boolean invertLeftSide = kInvertLeftSide;
    private boolean invertRightSide = kInvertRightSide;

    private CANBus canBus = new CANBus(kCANbusName);
    private int pigeonId = kPigeonId;

    // Simulation only values
    private double steerInertia = kSteerInertia;
    private double driveInertia = kDriveInertia;
    // Simulated voltage necessary to overcome friction
    private Voltage steerFrictionVoltage = Volts.of(kSteerFrictionVoltage);
    private Voltage driveFrictionVoltage = Volts.of(kDriveFrictionVoltage);

    private SwerveDrivetrainConstants drivetrainConstants;

    public SwerveDrivetrainConstants getDrivetrainConstants() {
        return drivetrainConstants;
    }

    private SwerveModuleConstantsFactory constantCreator;

    // Front left
    private int frontLeftDriveMotorId = kFrontLeftDriveMotorId;
    private int frontLeftSteerMotorId = kFrontLeftSteerMotorId;
    private int frontLeftEncoderId = kFrontLeftEncoderId;
    private Angle frontLeftEncoderOffset = Rotations.of(kFrontLeftEncoderOffset);

    private Distance frontLeftXPos = Inches.of(kFrontLeftXPos);
    private Distance frontLeftYPos = Inches.of(kFrontLeftYPos);

    // Front right
    private int frontRightDriveMotorId = kFrontLeftDriveMotorId;
    private int frontRightSteerMotorId = kFrontLeftSteerMotorId;
    private int frontRightEncoderId = kFrontLeftEncoderId;
    private Angle frontRightEncoderOffset = Rotations.of(kFrontLeftEncoderOffset);

    private Distance frontRightXPos = Inches.of(kFrontRightXPos);
    private Distance frontRightYPos = Inches.of(kFrontRightYPos);

    // Back left
    private int backLeftDriveMotorId = kBackLeftDriveMotorId;
    private int backLeftSteerMotorId = kBackLeftSteerMotorId;
    private int backLeftEncoderId = kBackLeftEncoderId;
    private Angle backLeftEncoderOffset = Rotations.of(kBackLeftEncoderOffset);

    private Distance backLeftXPos = Inches.of(kBackLeftXPos);
    private Distance backLeftYPos = Inches.of(kBackLeftYPos);

    // Back right
    private int backRightDriveMotorId = kBackRightDriveMotorId;
    private int backRightSteerMotorId = kBackRightSteerMotorId;
    private int backRightEncoderId = kBackRightEncoderId;
    private Angle backRightEncoderOffset = Rotations.of(kBackRightEncoderOffset);

    private Distance backRightXPos = Inches.of(kBackRightXPos);
    private Distance backRightYPos = Inches.of(kBackRightYPos);

    private SwerveModuleConstants frontLeft;
    private SwerveModuleConstants frontRight;
    private SwerveModuleConstants backLeft;
    private SwerveModuleConstants backRight;

    // Not too sure what this does yet
    private double targetHeading = 0;

    public SwerveModuleConstants[] getModules() {
        return new SwerveModuleConstants[] {frontLeft, frontRight, backLeft, backRight};
    }

    public SwerveConfig() {
        updateConfig();
    }

    public SwerveConfig updateConfig() {
        drivetrainConstants =
                new SwerveDrivetrainConstants()
                        .withCANBusName(canBus.getName())
                        .withPigeon2Id(pigeonId)
                        .withPigeon2Configs(pigeonConfigs);

        constantCreator =
                new SwerveModuleConstantsFactory()
                        .withDriveMotorGearRatio(driveGearRatio)
                        .withSteerMotorGearRatio(steerGearRatio)
                        .withWheelRadius(wheelRadius)
                        .withSlipCurrent(slipCurrent)
                        .withSteerMotorGains(steerGains)
                        .withDriveMotorGains(driveGains)
                        .withSteerMotorClosedLoopOutput(steerClosedLoopOutput)
                        .withDriveMotorClosedLoopOutput(driveClosedLoopOutput)
                        .withSpeedAt12Volts(speedAt12Volts)
                        .withSteerInertia(steerInertia)
                        .withDriveInertia(driveInertia)
                        .withSteerFrictionVoltage(steerFrictionVoltage)
                        .withDriveFrictionVoltage(driveFrictionVoltage)
                        .withFeedbackSource(SteerFeedbackType.FusedCANcoder)
                        .withCouplingGearRatio(coupleRatio)
                        .withDriveMotorInitialConfigs(driveInitialConfigs)
                        .withSteerMotorInitialConfigs(steerInitialConfigs);

        frontLeft =
                constantCreator.createModuleConstants(
                        frontLeftSteerMotorId,
                        frontLeftDriveMotorId,
                        frontLeftEncoderId,
                        frontLeftEncoderOffset,
                        frontLeftXPos,
                        frontLeftYPos,
                        invertLeftSide,
                        steerMotorReversed,
                        false);

        frontRight =
                constantCreator.createModuleConstants(
                        frontRightSteerMotorId,
                        frontRightDriveMotorId,
                        frontRightEncoderId,
                        frontRightEncoderOffset,
                        frontRightXPos,
                        frontRightYPos,
                        invertRightSide,
                        steerMotorReversed,
                        false);

        backLeft =
                constantCreator.createModuleConstants(
                        backLeftSteerMotorId,
                        backLeftDriveMotorId,
                        backLeftEncoderId,
                        backLeftEncoderOffset,
                        backLeftXPos,
                        backLeftYPos,
                        invertLeftSide,
                        steerMotorReversed,
                        false);

        backRight =
                constantCreator.createModuleConstants(
                        backRightSteerMotorId,
                        backRightDriveMotorId,
                        backRightEncoderId,
                        backRightEncoderOffset,
                        backRightXPos,
                        backRightYPos,
                        invertRightSide,
                        steerMotorReversed,
                        false);

        return this;
    }

    public SwerveConfig configEncoderOffsets(
            double frontLeft, double frontRight, double backLeft, double backRight) {
        frontLeftEncoderOffset = Rotations.of(frontLeft);
        frontRightEncoderOffset = Rotations.of(frontRight);
        backLeftEncoderOffset = Rotations.of(backLeft);
        backRightEncoderOffset = Rotations.of(backRight);
        return updateConfig();
    }
}
