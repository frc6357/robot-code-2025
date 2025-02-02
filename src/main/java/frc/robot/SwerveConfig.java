package frc.robot;

import lombok.Getter;
import lombok.Setter;
import static edu.wpi.first.units.Units.*;
import static frc.robot.Konstants.SwerveConstants.*;
import frc.robot.Konstants.SwerveConstants;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.*;

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

//"Straight outta Spectrum"

public class SwerveConfig {
    @Getter private final double kSimLoopPeriod = 0.005;
    @Getter private double robotWidth = Units.inchesToMeters(kChassisWidth);
    @Getter private double robotLength = Units.inchesToMeters(kChassisLength);

    @Getter private double maxAngularRate = 1.5*Math.PI;
    @Getter private double deadband = kDeadband;

    // Rotation Controller Constants
    @Getter private double maxAngularVelocity = 2 * Math.PI; // rad/s
    @Getter private double maxAngularAcceleration = Math.pow(maxAngularVelocity, 2); // rad/s^2
    @Getter private double kPRotationController = 8.0;
    @Getter private double kIRotationController = 0.0;
    @Getter private double kDRotationController = 0.2;
    @Getter private double rotationTolerance = (Math.PI / 360); // rads

    @Getter private double kPHoldController = 12.0;
    @Getter private double kIHoldController = 0.0;
    @Getter private double kDHoldController = 0.0;

    // Blue alliance sees forward as 0 degrees (toward red alliance wall)
    @Getter private final Rotation2d BlueAlliancePerspectiveRotation = Rotation2d.fromDegrees(0);
    // Red alliance sees forward as 180 degrees (toward blue alliance wall)
    @Getter private final Rotation2d RedAlliancePerspectiveRotation = Rotation2d.fromDegrees(180);

    // The steer motor uses any SwerveModule.SteerRequestType control request with the
    // output type specified by SwerveModuleConstants.SteerMotorClosedLoopOutput
    @Getter private Slot0Configs steerGains = SwerveConstants.kSteerGains;

    // When using closed-loop control, the drive motor uses the control
    // output type specified by SwerveModuleConstants.DriveMotorClosedLoopOutput
    @Getter private Slot0Configs driveGains = SwerveConstants.kDriveGains;

    // The stator current at which the wheels start to slip;
    // This needs to be tuned to your individual robot
    @Getter private Current slipCurrent = Amps.of(kSlipCurrentA);

    // Initial configs for the drive and steer motors and the CANcoder; these cannot be null.
    // Some configs will be overwritten; check the `with*InitialConfigs()` API documentation.
    @Getter private TalonFXConfiguration driveInitialConfigs = new TalonFXConfiguration();


    @Getter
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

    @Getter private CANcoderConfiguration cancoderInitialConfigs = new CANcoderConfiguration();
    // Configs for the Pigeon 2; leave this null to skip applying Pigeon 2 configs
    @Getter private Pigeon2Configuration pigeonConfigs = new Pigeon2Configuration();

    @Getter @Setter private LinearVelocity speedAt12Volts = MetersPerSecond.of(kSpeedAt12VoltsMps);

    @Getter private double coupleRatio = kCoupleRatio;

    @Getter @Setter private double driveGearRatio = kDriveGearRatio;
    @Getter @Setter private double steerGearRatio =kSteerGearRatio;

    @Getter @Setter private Distance wheelRadius = Inches.of(kWheelRadius);

    @Getter @Setter private boolean steerMotorReversed = kSteerMotorReversed;
    @Getter @Setter private boolean invertLeftSide = kInvertLeftSide;
    @Getter @Setter private boolean invertRightSide = kInvertRightSide;

    @Getter @Setter private CANBus canBus = new CANBus(kCANbusName);
    @Getter private int pigeonId = kPigeonId;

    // Simulation only values
    @Getter private double steerInertia = kSteerInertia;
    @Getter private double driveInertia = kDriveInertia;
    // Simulated voltage necessary to overcome friction
    @Getter private Voltage steerFrictionVoltage = Volts.of(kSteerFrictionVoltage);
    @Getter private Voltage driveFrictionVoltage = Volts.of(kDriveFrictionVoltage);

    @Getter private SwerveDrivetrainConstants drivetrainConstants;

    @Getter private SwerveModuleConstantsFactory constantCreator;

    // Front left
    @Getter private int frontLeftDriveMotorId = kFrontLeftDriveMotorId;
    @Getter private int frontLeftSteerMotorId = kFrontLeftSteerMotorId;
    @Getter private int frontLeftEncoderId = kFrontLeftEncoderId;
    @Getter private Angle frontLeftEncoderOffset = Rotations.of(kFrontLeftEncoderOffset);

    @Getter private Distance frontLeftXPos = Inches.of(kFrontLeftXPos);
    @Getter private Distance frontLeftYPos = Inches.of(kFrontLeftYPos);

    // Front right
    @Getter private int frontRightDriveMotorId = kFrontLeftDriveMotorId;
    @Getter private int frontRightSteerMotorId = kFrontLeftSteerMotorId;
    @Getter private int frontRightEncoderId = kFrontLeftEncoderId;
    @Getter private Angle frontRightEncoderOffset = Rotations.of(kFrontLeftEncoderOffset);

    @Getter private Distance frontRightXPos = Inches.of(kFrontRightXPos);
    @Getter private Distance frontRightYPos = Inches.of(kFrontRightYPos);

    // Back left
    @Getter private int backLeftDriveMotorId = kBackLeftDriveMotorId;
    @Getter private int backLeftSteerMotorId = kBackLeftSteerMotorId;
    @Getter private int backLeftEncoderId = kBackLeftEncoderId;
    @Getter private Angle backLeftEncoderOffset = Rotations.of(kBackLeftEncoderOffset);

    @Getter private Distance backLeftXPos = Inches.of(kBackLeftXPos);
    @Getter private Distance backLeftYPos = Inches.of(kBackLeftYPos);

    // Back right
    @Getter private int backRightDriveMotorId = kBackRightDriveMotorId;
    @Getter private int backRightSteerMotorId = kBackRightSteerMotorId;
    @Getter private int backRightEncoderId = kBackRightEncoderId;
    @Getter private Angle backRightEncoderOffset = Rotations.of(kBackRightEncoderOffset);

    @Getter private Distance backRightXPos = Inches.of(kBackRightXPos);
    @Getter private Distance backRightYPos = Inches.of(kBackRightYPos);

    @Getter private SwerveModuleConstants frontLeft;
    @Getter private SwerveModuleConstants frontRight;
    @Getter private SwerveModuleConstants backLeft;
    @Getter private SwerveModuleConstants backRight;

    // Not too sure what this does yet
    @Getter private double targetHeading = 0;

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
