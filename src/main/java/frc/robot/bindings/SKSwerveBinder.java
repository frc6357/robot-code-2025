package frc.robot.bindings;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static frc.robot.Konstants.OIConstants.kJoystickDeadband;
import static frc.robot.Konstants.OIConstants.kMaxFullSpeedElevatorHeight;
import static frc.robot.Konstants.OIConstants.kSlowModePercent;
import static frc.robot.Ports.DriverPorts.kDriveFn;
import static frc.robot.Ports.DriverPorts.kResetGyroPos;
import static frc.robot.Ports.DriverPorts.kRobotCentricMode;
import static frc.robot.Ports.DriverPorts.kSlowMode;
import static frc.robot.Ports.DriverPorts.kTranslationXPort;
import static frc.robot.Ports.DriverPorts.kTranslationYPort;
import static frc.robot.Ports.DriverPorts.kVelocityOmegaPort;

import java.util.Optional;
import java.util.function.Supplier;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

// Used for binding buttons to drive actions
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.RobotContainer;
import frc.robot.TunerConstants;
import frc.robot.preferences.Pref;
import frc.robot.preferences.SKPreferences;
import frc.robot.subsystems.SK25Elevator;
// Adds the Swerve subsystem for construction
import frc.robot.subsystems.SKSwerve;
// Filters used for input types (specifically Axis inputs)
import frc.robot.utils.filters.DeadbandFilter;
import frc.robot.utils.filters.DriveStickFilter;
import frc.robot.utils.filters.Filter;
import lombok.Getter;

/* Leftover code from Phoenix's configureBindings():
    joystick.a().whileTrue(m_swerve.get().applyRequest(() -> brake));
    joystick.b().whileTrue(m_swerve.get().applyRequest(() ->
        point.withModuleDirection(new Rotation2d(-joystick.getLeftY(), -joystick.getLeftX()))
    ));
    Run SysId routines when holding back/start and X/Y.
    Note that each routine should be run exactly once in a single log.
    joystick.back().and(joystick.y()).whileTrue(m_swerve.get().sysIdDynamic(Direction.kForward));
    joystick.back().and(joystick.x()).whileTrue(m_swerve.get().sysIdDynamic(Direction.kReverse));
    joystick.start().and(joystick.y()).whileTrue(m_swerve.get().sysIdQuasistatic(Direction.kForward));
    joystick.start().and(joystick.x()).whileTrue(m_swerve.get().sysIdQuasistatic(Direction.kReverse));

 */

public class SKSwerveBinder implements CommandBinder{
    Optional<SKSwerve>  m_drive;
    DriveStickFilter translationXFilter;
    DriveStickFilter translationYFilter;
    DriveStickFilter rotationFilter;

     Pref<Double> driverTranslationSlewPref = SKPreferences.attach("driverTranslSlew", 1.5)
                 .onChange((newValue) -> {
                     translationXFilter.setSlewRate(newValue);
                     translationYFilter.setSlewRate(newValue);
                 });
    

    
    Pref<Double> driverRotationSlewPref = SKPreferences.attach("driverRotSlew", 4.0)
                .onChange((newValue) -> {
                    rotationFilter.setSlewRate(newValue);
                });

    @Getter private double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
    @Getter private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation per second max angular velocity

    // Driver Buttons
    public final Trigger fn = kDriveFn.button;
    public final Trigger noFn = fn.negate();

    private final Trigger robotCentric = kRobotCentricMode.button.and(fn);
    
    private final Trigger slowmode = kSlowMode.button.and(fn);
    private final Trigger resetButton = kResetGyroPos.button;


    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDeadband(kJoystickDeadband).withRotationalDeadband(kJoystickDeadband)
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors
    private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
    private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();


    public SKSwerveBinder(Optional<SKSwerve> m_drive, Optional<SK25Elevator> m_elevator) {
        this.m_drive = m_drive;
        // apply acceleration limits based on the elevator height if the elevator is present
        if(m_elevator.isPresent())
        {
            this.translationXFilter = new DriveStickFilter(
                MaxSpeed, 
                elevatorHeightDriveScalar(
                    driverTranslationSlewPref.get(), 
                    m_elevator.get().getCurrentHeightMotorRotations()),
                kJoystickDeadband);
            this.translationYFilter = new DriveStickFilter(
                MaxSpeed, 
                elevatorHeightDriveScalar(
                    driverTranslationSlewPref.get(), 
                    m_elevator.get().getCurrentHeightMotorRotations()), 
                kJoystickDeadband);

            //rotation filter dosnt need to be scaled by elevator height since it dosn't affect the
            //magnitude of the robot's velocity vecotr.
            this.rotationFilter = new DriveStickFilter(
                MaxAngularRate, 
                driverRotationSlewPref.get(), 
                kJoystickDeadband);
        }
        //if the elevator is absent, apply the default slew rates
        else
        {
            this.translationXFilter = new DriveStickFilter(
                MaxSpeed, 
                driverTranslationSlewPref.get(),
                kJoystickDeadband);
            this.translationYFilter = new DriveStickFilter(
                MaxSpeed, 
                driverTranslationSlewPref.get(), 
                kJoystickDeadband);

            this.rotationFilter = new DriveStickFilter(
                MaxAngularRate, 
                driverRotationSlewPref.get(), 
                kJoystickDeadband);
        }
    }


    /** Adds the slew rate required to accelerate at the maximum possible value without tipping over. Uses
     * the elevator height to add a slew rate to the current rate by increasing or decreasing the added
     * value as a percentage.
     * @param slewaRate The default slew rate (limit in the change in input value from the joystick) of
     * the drive.
     * @param elevatorHeight The current height of the elevator in motor rotations.
     * @return The slew rate of the swerve with the elevator height accounted for.
     */
    public double elevatorHeightDriveScalar(double slewRate, Supplier<Double> elevatorHeight)
    {
        if (elevatorHeight.get() <= kMaxFullSpeedElevatorHeight)
        {
            return slewRate;
        }
        else
        {
            double scaledElevatorRate = (elevatorHeight.get() / 13.5) - (kMaxFullSpeedElevatorHeight / 13.5);
            return scaledElevatorRate + slewRate;
        }
    }


    @Override
    public void bindButtons() {
        if (m_drive.isPresent())
        {
            SKSwerve drivetrain = m_drive.get();
            
            // Sets filters for driving axes
             kTranslationXPort.setFilter(translationXFilter);

             kTranslationYPort.setFilter(translationYFilter);
            
            kVelocityOmegaPort.setFilter(rotationFilter);

            slowmode.
                onTrue(new InstantCommand(() -> {setGainCommand(kSlowModePercent);}, drivetrain))
                .onFalse(new InstantCommand(() -> {setGainCommand(1);}, drivetrain));

            // Resets gyro angles
            resetButton.onTrue(new InstantCommand(() -> {drivetrain.seedFieldCentric();} ));

            // Default command for driving
        //     drive.setDefaultCommand(
        //         new DefaultSwerveCommand(
        //             () -> kTranslationXPort.getFilteredAxis(),
        //             () -> kTranslationYPort.getFilteredAxis(),
        //             () -> kVelocityOmegaPort.getFilteredAxis(),
        //             () -> {return true;}, 
        //             drive));

            drivetrain.setDefaultCommand(
                // Drivetrain will execute this command periodically
                drivetrain.applyRequest(() -> {
                    return drive.withVelocityX(kTranslationXPort.getFilteredAxis()) // Drive forward with negative Y (forward)
                        .withVelocityY(kTranslationYPort.getFilteredAxis()) // Drive left with negative X (left)
                        .withRotationalRate(kVelocityOmegaPort.getFilteredAxis()); // Drive counterclockwise with negative X (left)
                })
            );
        }
    }

    /**
     * Sets the gains on the filters for the joysticks
     * 
     * @param percent
     *            The percent value of the full output that should be allowed (value
     *            should be between 0 and 1)
     */
    public void setGainCommand(double percent)
    {
        Filter translation = new DeadbandFilter(kJoystickDeadband, percent);
        kTranslationXPort.setFilter(translation);
        kTranslationYPort.setFilter(translation);

     
        Filter rotation = new DeadbandFilter(kJoystickDeadband,percent);
        kVelocityOmegaPort.setFilter(rotation);
  
    }

}
