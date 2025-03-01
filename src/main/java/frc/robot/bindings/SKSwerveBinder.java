package frc.robot.bindings;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static frc.robot.Konstants.OIConstants.kJoystickDeadband;
import static frc.robot.Konstants.OIConstants.kMaxFullSpeedElevatorHeight;
import static frc.robot.Konstants.SwerveConstants.kSlowModePercentage;
import static frc.robot.Ports.DriverPorts.kDriveFn;
import static frc.robot.Ports.DriverPorts.kResetGyroPos;
//import static frc.robot.Ports.DriverPorts.kRobotCentricMode;
import static frc.robot.Ports.DriverPorts.kSlowMode;
import static frc.robot.Ports.DriverPorts.kTranslationXPort;
import static frc.robot.Ports.DriverPorts.kTranslationYPort;
import static frc.robot.Ports.DriverPorts.kVelocityOmegaPort;

import java.util.Optional;
import java.util.function.Supplier;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
// Used for binding buttons to drive actions
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.TunerConstants;
import frc.robot.preferences.Pref;
import frc.robot.preferences.SKPreferences;
import frc.robot.subsystems.SK25Elevator;
// Adds the Swerve subsystem for construction
import frc.robot.subsystems.SKSwerve;
// Filter used for input types (specifically Axis inputs)
import frc.robot.utils.filters.DriveStickFilter;

public class SKSwerveBinder implements CommandBinder{
    Optional<SKSwerve>  m_drive;
    DriveStickFilter translationXFilter;
    DriveStickFilter translationYFilter;
    DriveStickFilter rotationFilter;
    boolean slowModeStatus;

    //Allow alterable slew rates from the dashboard.
     Pref<Double> driverTranslationSlewPref = SKPreferences.attach("driverTranslSlew", 1.5)
                 .onChange((newValue) -> {
                     translationXFilter.setSlewRate(newValue);
                     translationYFilter.setSlewRate(newValue);
                 });
    

    //Allow alterable slew rates from the dashboard.
    Pref<Double> driverRotationSlewPref = SKPreferences.attach("driverRotSlew", 4.0)
                .onChange((newValue) -> {
                    rotationFilter.setSlewRate(newValue);
                });

    //Define the max speeds of the drivetrain
    private double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
    private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation per second max angular velocity

    // Driver Buttons
    //The function button enables button combinations which occur only when both the function and the other
    //specified button are pressed.
    public final Trigger fn = kDriveFn.button;
    public final Trigger noFn = fn.negate();

    //Other driver buttons
    //private final Trigger robotCentric = kRobotCentricMode.button.and(fn);
    private final Trigger slowmode = kSlowMode.button.and(fn);
    private final Trigger resetButton = kResetGyroPos.button;

    //Create swerve drive request objects for applicatoin to the drivetrain
    private final SwerveRequest.FieldCentric feildCentricDrive = new SwerveRequest.FieldCentric()
            .withDeadband(kJoystickDeadband).withRotationalDeadband(kJoystickDeadband)
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors
    
    final SwerveRequest.RobotCentric robotCentricDrive = new SwerveRequest.RobotCentric();
            // this.applyRequest(() ->
            //     robotCentricDrive.withVelocityX(speeds.vxMetersPerSecond) // Drive forward with negative Y (forward)
            //         .withVelocityY(speeds.vyMetersPerSecond) // Drive left with negative X (left)
            //         .withRotationalRate(speeds.omegaRadiansPerSecond)); // Drive counterclockwise with negative X (left)
   
    //private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
    //private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();


    public SKSwerveBinder(Optional<SKSwerve> m_drive, Optional<SK25Elevator> m_elevator) {
        this.m_drive = m_drive;
        //slow mode is initialy deactivated
        this.slowModeStatus = false;

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

    /** Sets the slow mode status by changing the slowModeStatus boolean variable.
     * @param status The status to set the slow mode to.
     */
    public void setSlowMode(boolean status)
    {
        SmartDashboard.putBoolean("slowModeStatus", status);
        slowModeStatus = status;

        //If slowMode is enabled, drive at the slowMode speed.
        if (slowModeStatus)
        {
            this.translationXFilter.setMaxSpeed(MaxSpeed * kSlowModePercentage);
            this.translationYFilter.setMaxSpeed(MaxSpeed * kSlowModePercentage);
            this.rotationFilter.setMaxSpeed(MaxAngularRate * kSlowModePercentage);
        }
        //If slow mode is not enabled, drive at the default speed.
        else
        {
            this.translationXFilter.setMaxSpeed(MaxSpeed);
            this.translationYFilter.setMaxSpeed(MaxSpeed);
            this.rotationFilter.setMaxSpeed(MaxAngularRate);
        }
    }


    @Override
    public void bindButtons()
    {
        if (!m_drive.isPresent())
        {
            return;
        }
        
        //set to false by defualt
        setSlowMode(slowModeStatus);

        SKSwerve drivetrain = m_drive.get();
        
        // Sets filters for driving axes
        kTranslationXPort.setFilter(translationXFilter);
        kTranslationYPort.setFilter(translationYFilter);
        kVelocityOmegaPort.setFilter(rotationFilter);

        //Apply slow mode if activated
        slowmode.onTrue(new InstantCommand(() -> setSlowMode(true)));
        slowmode.onFalse(new InstantCommand(() -> setSlowMode(false)));
        
        // Resets gyro angles / robot oreintation
        resetButton.onTrue(new InstantCommand(() -> {drivetrain.seedFieldCentric();} ));

        drivetrain.setDefaultCommand(
            // Drivetrain will execute this command periodically
            drivetrain.applyRequest(() -> {
                return feildCentricDrive.withVelocityX(kTranslationXPort.getFilteredAxis()) // Drive forward with negative Y (forward)
                    .withVelocityY(kTranslationYPort.getFilteredAxis()) // Drive left with negative X (left)
                    .withRotationalRate(kVelocityOmegaPort.getFilteredAxis()); // Drive counterclockwise with negative X (left)
            })
        );
    }
}