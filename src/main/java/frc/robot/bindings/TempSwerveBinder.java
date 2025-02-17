// package frc.robot.bindings;

// import static frc.robot.Konstants.OIConstants.kDriveCoeff;
// import static frc.robot.Konstants.OIConstants.kJoystickDeadband;
// import static frc.robot.Konstants.OIConstants.kRotationCoeff;
// import static frc.robot.Konstants.OIConstants.kSlowModePercent;
// import static frc.robot.Ports.DriverPorts.kDriveFn;
// import static frc.robot.Ports.DriverPorts.kResetGyroPos;
// // import static frc.robot.Ports.DriverPorts.kRotateLeft;
// // import static frc.robot.Ports.DriverPorts.kRotateRight;
// // import static frc.robot.Ports.DriverPorts.kRotateSource;
// // import static frc.robot.Ports.DriverPorts.kRotateSpeaker;
// import static frc.robot.Ports.DriverPorts.kSlowMode;
// import static frc.robot.Ports.DriverPorts.kTranslationXPort;
// import static frc.robot.Ports.DriverPorts.kTranslationYPort;
// import static frc.robot.Ports.DriverPorts.kVelocityOmegaPort;

// import java.util.Optional;

// // Used for binding buttons to drive actions
// import edu.wpi.first.wpilibj2.command.InstantCommand;
// import edu.wpi.first.wpilibj2.command.button.Trigger;
// import frc.robot.Konstants.SwerveConstants;
// import frc.robot.commands.TempSwerveCommand;
// // Adds the Swerve subsystem for construction
// import frc.robot.subsystems.TempSwerve;
// // Filters used for input types (specifically Axis inputs)
// import frc.robot.utils.filters.CubicDeadbandFilter;
// import frc.robot.utils.filters.Filter;


// public class TempSwerveBinder implements CommandBinder{
//     Optional<TempSwerve>  m_swerve;

//     // Driver Buttons
//     public final Trigger fn = kDriveFn.button;
//     public final Trigger noFn = fn.negate();
    
//     private final Trigger slowmode = kSlowMode.button.and(fn);
//     private final Trigger resetButton = kResetGyroPos.button;



//     public TempSwerveBinder(Optional<TempSwerve> m_swerve)
//     { 
//         this.m_swerve = m_swerve;
//     }


//     @Override
//     public void bindButtons() {
//         if (m_swerve.isPresent())
//         {
//             TempSwerve swerve = m_swerve.get();

//             // Sets filters for driving axes
//             kTranslationXPort.setFilter(new CubicDeadbandFilter(kDriveCoeff,
//                 kJoystickDeadband, SwerveConstants.kMaxDriveSpeedMetersPerSecond, true));

//             kTranslationYPort.setFilter(new CubicDeadbandFilter(kDriveCoeff,
//                 kJoystickDeadband, SwerveConstants.kMaxDriveSpeedMetersPerSecond, true));
            
//             kVelocityOmegaPort.setFilter(new CubicDeadbandFilter(kRotationCoeff, kJoystickDeadband,
//                 Math.toRadians(SwerveConstants.kMaxModuleAngularSpeedDegreesPerSecond), true));

//             slowmode.
//                 onTrue(new InstantCommand(() -> {setGainCommand(kSlowModePercent);}, swerve))
//                 .onFalse(new InstantCommand(() -> {setGainCommand(1);}, swerve));

//             // Resets gyro angles
//             //resetButton.onTrue(swerve.reorientForward());

//             // Default command for driving
//             swerve.setDefaultCommand(
//                 new TempSwerveCommand(                //TODO: uncomment when used again
//                     swerve,
//                     () -> kTranslationXPort.getFilteredAxis(),
//                     () -> kTranslationYPort.getFilteredAxis(),
//                     () -> kVelocityOmegaPort.getFilteredAxis()
//                     ));
//         }
//     }

//     /**
//      * Sets the gains on the filters for the joysticks
//      * 
//      * @param percent
//      *            The percent value of the full output that should be allowed (value
//      *            should be between 0 and 1)
//      */
//     public void setGainCommand(double percent)
//     {
//         Filter translation = new CubicDeadbandFilter(kDriveCoeff, kJoystickDeadband,
//             SwerveConstants.kMaxDriveSpeedMetersPerSecond * percent, true);
//         kTranslationXPort.setFilter(translation);
//         kTranslationYPort.setFilter(translation);

     
//         Filter rotation = new CubicDeadbandFilter(kDriveCoeff, kJoystickDeadband,
//             Math.toRadians(SwerveConstants.kMaxRotationDegreesPerSecond) * percent, true);
//         kVelocityOmegaPort.setFilter(rotation);
  
//     }

// }
