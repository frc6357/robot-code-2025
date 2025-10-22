package frc.robot.bindings;

import static frc.lib.drive.FollowAutoPathSequence.NewPathSequenceCommand;
import static frc.robot.Konstants.OIConstants.kJoystickDeadband;
import static frc.robot.Ports.DriverPorts.kAlignToReef;
import static frc.robot.Ports.DriverPorts.kDriveFn;
import static frc.robot.Ports.DriverPorts.kFastMode;
import static frc.robot.Ports.DriverPorts.kResetGyroPos;
import static frc.robot.Ports.DriverPorts.kRobotCentricMode;
import static frc.robot.Ports.DriverPorts.kSimulateCollision;
import static frc.robot.Ports.DriverPorts.kSlowMode;
import static frc.robot.Ports.DriverPorts.kTranslationXPort;
import static frc.robot.Ports.DriverPorts.kTranslationYPort;
import static frc.robot.Ports.DriverPorts.kVelocityOmegaPort;

import java.util.Optional;

import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.lib.drive.FollowPath;
import frc.lib.preferences.Pref;
import frc.lib.preferences.SKPreferences;
import frc.lib.utils.filters.DriveStickFilter;
import frc.robot.subsystems.drive.DriveRequests;
import frc.robot.subsystems.drive.SKSwerve;

public class SKSwerveBinder implements CommandBinder{
    Optional<SKSwerve>  m_drive;
    DriveStickFilter translationXFilter;
    DriveStickFilter translationYFilter;
    DriveStickFilter rotationFilter;
    boolean slowModeStatus;

    //Allow alterable slew rates from the dashboard.
     Pref<Double> driverTranslationSlewPref = SKPreferences.attach("driverTranslSlew", 4.0)
                 .onChange((newValue) -> {
                     translationXFilter.setSlewRate(newValue);
                     translationYFilter.setSlewRate(newValue);
                 });
    

    //Allow alterable slew rates from the dashboard.
    Pref<Double> driverRotationSlewPref = SKPreferences.attach("driverRotSlew", 4.0)
                .onChange((newValue) -> {
                    rotationFilter.setSlewRate(newValue);
                });

    // Driver Buttons
    //The function button enables button combinations which occur only when both the function and the other
    //specified button are pressed.
    public final Trigger fn = kDriveFn.button;
    public final Trigger noFn = fn.negate();

    //Other driver buttons
    private final Trigger robotCentric = kRobotCentricMode.button;
    private final Trigger slowmode = kSlowMode.button;
    private final Trigger resetButton = kResetGyroPos.button;
    private final Trigger fastmode = kFastMode.button;
    
    /**
     * Experimental Autonomous Triggers
     */
    private final Trigger simulateCollision = kSimulateCollision.button;
    private final Trigger runAuto = robotCentric;
    private Trigger pathfindToReef = kAlignToReef.button;


    
    final SwerveRequest.RobotCentric robotCentricDrive = new SwerveRequest.RobotCentric();
    //private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
    //private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();


    public SKSwerveBinder(Optional<SKSwerve> m_drive) {
        this.m_drive = m_drive;

        this.translationXFilter = new DriveStickFilter(
            driverTranslationSlewPref.get(),
            kJoystickDeadband);
        this.translationYFilter = new DriveStickFilter(
            driverTranslationSlewPref.get(), 
            kJoystickDeadband);
        this.rotationFilter = new DriveStickFilter(
            driverRotationSlewPref.get(), 
            kJoystickDeadband);
    }

    @Override
    public void bindButtons()
    {
        if (!m_drive.isPresent())
        {
            return;
        }

        SKSwerve drive = m_drive.get();
        
        /**
         * Experimental Pathfinding Code
         */
        pathfindToReef.whileTrue(FollowPath.PathfindThenFollowPathCommand("Seamless3GP"));

        runAuto.whileTrue(NewPathSequenceCommand("DriftlessBargeL4(3GP)"));

        simulateCollision.onTrue(new InstantCommand(() -> {drive.simulateCollision();} ));
        /**
         * End Experimental Pathfinding Code
         */

        // Sets filters for driving axes
        // kTranslationXPort.setFilter(translationXFilter);
        // kTranslationYPort.setFilter(translationYFilter);
        // kVelocityOmegaPort.setFilter(rotationFilter);

        // robotCentric.whileTrue(
        //     drivetrain.applyRequest(() -> {
        //         return robotCentricDrive.withVelocityX(applyGains(-MaxSpeed * kTranslationXPort.getFilteredAxis(), kSlowModePercent)) // Drive forward with negative Y (forward)
        //             .withVelocityY(applyGains(-MaxSpeed * kTranslationYPort.getFilteredAxis(), kSlowModePercent)) // Drive left with negative X (left)
        //             .withRotationalRate(applyGains(MaxSpeed * -1.0 * kVelocityOmegaPort.getFilteredAxis(), kSlowModePercent)); // Drive counterclockwise with negative X (left)
        //     })
        // );


        //Apply slow mode if activated
        // slowmode.onTrue(new InstantCommand(() -> setSlowMode(true)));
        // slowmode.onFalse(new InstantCommand(() -> setSlowMode(false)));
        
        // Resets gyro angles / robot oreintation
        resetButton.onTrue(new InstantCommand(() -> {drive.resetOrientation();} ));

        drive.setDefaultCommand(
            drive.followSwerveRequestCommand(
                DriveRequests.teleopRequest, 
                DriveRequests.getTeleopRequestUpdater(
                        () -> -kTranslationXPort.getFilteredAxis(), 
                        () -> -kTranslationYPort.getFilteredAxis(), 
                        () -> -kVelocityOmegaPort.getFilteredAxis(), 
                        () -> slowmode.getAsBoolean(), 
                        () -> fastmode.getAsBoolean())
            )
        );

        // drive.getDrivetrain().setDefaultCommand(
        //     // Drivetrain will execute this command periodically
        //     drive.getDrivetrain().applyRequest(() -> {
        //         return feildCentricDrive.withVelocityX(applyGains(-kMaxSpeed.in(MetersPerSecond) * kTranslationXPort.getFilteredAxis(), kSlowModePercent)) // Drive forward with negative Y (forward)
        //             .withVelocityY(applyGains(-kMaxSpeed.in(MetersPerSecond) * kTranslationYPort.getFilteredAxis(), kSlowModePercent)) // Drive left with negative X (left)
        //             .withRotationalRate(applyGains(kMaxAngularRate.in(RadiansPerSecond) * -1.0 * kVelocityOmegaPort.getFilteredAxis(), kSlowModeRotationPercent)); // Drive counterclockwise with negative X (left)
        //     })
        // );



    }
}