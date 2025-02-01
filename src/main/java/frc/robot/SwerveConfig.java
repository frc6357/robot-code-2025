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
import com.ctre.phoenix6.swerve.SwerveModuleConstants.ClosedLoopOutputType;
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

    private SwerveDrivetrainConstants drivetrainConstants;

    private SwerveModuleConstantsFactory constantCreator;

    
}
