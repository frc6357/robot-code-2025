package frc.robot.bindings;

import static frc.robot.Konstants.OIConstants.kDriveCoeff;
import static frc.robot.Konstants.OIConstants.kJoystickDeadband;
import static frc.robot.Konstants.OIConstants.kRotationCoeff;
import static frc.robot.Konstants.OIConstants.kSlowModePercent;
import static frc.robot.Ports.DriverPorts.kResetGyroPos;
import static frc.robot.Ports.DriverPorts.kRobotCentricMode;
// import static frc.robot.Ports.DriverPorts.kRotateLeft;
// import static frc.robot.Ports.DriverPorts.kRotateRight;
// import static frc.robot.Ports.DriverPorts.kRotateSource;
// import static frc.robot.Ports.DriverPorts.kRotateSpeaker;
import static frc.robot.Ports.DriverPorts.kSlowMode;
import static frc.robot.Ports.DriverPorts.kTranslationXPort;
import static frc.robot.Ports.DriverPorts.kTranslationYPort;
import static frc.robot.Ports.DriverPorts.kVelocityOmegaPort;
import static frc.robot.Ports.DriverPorts.kDriveFn;

// Filters used for input types (specifically Axis inputs)
import frc.robot.utils.filters.CubicDeadbandFilter;
import frc.robot.utils.filters.Filter;

// Used for binding buttons to drive actions
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Konstants.SwerveConstants;
import frc.robot.commands.DefaultSwerveCommand;
// Adds the Swerve subsystem for construction
import frc.robot.subsystems.swerve.SK25Swerve;

import java.util.Optional;


public class SK25DriveBinder implements CommandBinder{
    Optional<SK25Swerve>  m_drive;

    // Driver Buttons
    public final Trigger fn = kDriveFn.button;
    public final Trigger noFn = fn.negate();

    private final Trigger robotCentric = kRobotCentricMode.button.and(fn);
    
    private final Trigger slowmode = kSlowMode.button.and(fn);
    private final Trigger resetButton = kResetGyroPos.button;



    public SK25DriveBinder(Optional<SK25Swerve> m_drive) { //TODO: Add more subsytems later for command binding
        this.m_drive = m_drive;
    }


    @Override
    public void bindButtons() {
        if (m_drive.isPresent())
        {
            SK25Swerve drive = m_drive.get();

            // Sets filters for driving axes
            kTranslationXPort.setFilter(new CubicDeadbandFilter(kDriveCoeff,
                kJoystickDeadband, SwerveConstants.kMaxDriveSpeedMetersPerSecond, true));

            kTranslationYPort.setFilter(new CubicDeadbandFilter(kDriveCoeff,
                kJoystickDeadband, SwerveConstants.kMaxDriveSpeedMetersPerSecond, true));
            
            kVelocityOmegaPort.setFilter(new CubicDeadbandFilter(kRotationCoeff, kJoystickDeadband,
                Math.toRadians(SwerveConstants.kMaxModuleAngularSpeedDegreesPerSecond), true));

            slowmode.
                onTrue(new InstantCommand(() -> {setGainCommand(kSlowModePercent);}, drive))
                .onFalse(new InstantCommand(() -> {setGainCommand(1);}, drive));

            // Resets gyro angles
            resetButton.onTrue(drive.reorientForward());

            // Default command for driving
            drive.setDefaultCommand(
                new DefaultSwerveCommand(
                    () -> kTranslationXPort.getFilteredAxis(),
                    () -> kTranslationYPort.getFilteredAxis(),
                    () -> kVelocityOmegaPort.getFilteredAxis(),
                    () -> {return true;}, 
                    drive));
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
        Filter translation = new CubicDeadbandFilter(kDriveCoeff, kJoystickDeadband,
            SwerveConstants.kMaxDriveSpeedMetersPerSecond * percent, true);
        kTranslationXPort.setFilter(translation);
        kTranslationYPort.setFilter(translation);

     
        Filter rotation = new CubicDeadbandFilter(kDriveCoeff, kJoystickDeadband,
            Math.toRadians(SwerveConstants.kMaxRotationDegreesPerSecond) * percent, true);
        kVelocityOmegaPort.setFilter(rotation);
  
    }

}
