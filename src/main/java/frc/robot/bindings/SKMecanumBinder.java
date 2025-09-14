package frc.robot.bindings;

import static frc.robot.Konstants.IOConstants.kJoystickDeadband;
import static frc.robot.Konstants.IOConstants.kSlowModePercent;
import static frc.robot.Konstants.IOConstants.kSlowModeRotationPercent;
import static frc.robot.Konstants.MecanumDriveConstants.kMaxAngularSpeed;
import static frc.robot.Konstants.MecanumDriveConstants.kMaxSpeed;
import static frc.robot.Ports.DriverPorts.kResetGyroPos;
import static frc.robot.Ports.DriverPorts.kRobotCentricMode;
import static frc.robot.Ports.DriverPorts.kSlowMode;
import static frc.robot.Ports.DriverPorts.kTranslationXPort;
import static frc.robot.Ports.DriverPorts.kTranslationYPort;
import static frc.robot.Ports.DriverPorts.kVelocityOmegaPort;

import java.util.Optional;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.DriveCommand;
import frc.robot.preferences.Pref;
import frc.robot.preferences.SKPreferences;
import frc.robot.subsystems.SKMecanumDrive;
import frc.robot.utils.filters.DriveStickFilter;

// Lots of this class is adapted from 2025-robot-code's SKSwerveBinder class


public class SKMecanumBinder implements CommandBinder {
    Optional<SKMecanumDrive> m_drive;
    DriveStickFilter translationXFilter;
    DriveStickFilter translationYFilter;
    DriveStickFilter rotationFilter;
    boolean slowModeStatus;
    DriveCommand fieldCentricDrive;
    DriveCommand robotCentricDrive;

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

    private Trigger robotCentric = kRobotCentricMode.button;
    private Trigger slowmode = kSlowMode.button;
    private Trigger resetButton = kResetGyroPos.button;

    public SKMecanumBinder(Optional<SKMecanumDrive> m_drive) {
        this.m_drive = m_drive;
        this.slowModeStatus = false;

        this.translationXFilter = new DriveStickFilter(
                kMaxSpeed, 
                driverTranslationSlewPref.get(),
                kJoystickDeadband);
        this.translationYFilter = new DriveStickFilter(
                kMaxSpeed, 
                driverTranslationSlewPref.get(), 
                kJoystickDeadband);

        this.rotationFilter = new DriveStickFilter(
                kMaxAngularSpeed, 
                driverRotationSlewPref.get(), 
                kJoystickDeadband);
    }


    @Override
    public void bindButtons() {
        if(!m_drive.isPresent()) {
            return;
        }
        SKMecanumDrive drivetrain = m_drive.get();
        
        // Set to false by defualt
        setSlowMode(slowModeStatus);
        //Apply slow mode if activated
        slowmode.onTrue(new InstantCommand(() -> setSlowMode(true)));
        slowmode.onFalse(new InstantCommand(() -> setSlowMode(false)));

        // Sets filters for driving axes
        kTranslationXPort.setFilter(translationXFilter);
        kTranslationYPort.setFilter(translationYFilter);
        kVelocityOmegaPort.setFilter(rotationFilter);

        fieldCentricDrive = new DriveCommand(
            drivetrain, 
            () -> getVelX(), 
            () -> getVelY(), 
            () -> getVelOmega(), 
            ()-> true);
        
        robotCentricDrive = new DriveCommand(
            drivetrain, 
            () -> getVelX(), 
            () -> getVelY(), 
            () -> getVelOmega(), 
            ()-> false);
        
        robotCentric.whileTrue(new InstantCommand(() -> robotCentricDrive.run()));
        drivetrain.setDefaultCommand(new InstantCommand(() -> fieldCentricDrive.run()));

        resetButton.onTrue(new InstantCommand(() -> drivetrain.resetRotation()));
    }

    /** Sets the slow mode status by changing the slowModeStatus boolean variable.
     * @param status The status to set the slow mode to.
     */
    public void setSlowMode(boolean status)
    {
        SmartDashboard.putBoolean("slowModeStatus", status);
        slowModeStatus = status;
    }

    /**
     * Checks to see if slowmode is enabled. If enabled, applies the fed gain percentage to the axis.
     * @param axis The axis' value
     * @param slowPercent The percentage to amplyify/slow by
     * @return The adjusted axis value
     */
    public double applyGains(double axis, double slowPercent)
    {
        if (slowModeStatus)
        {
            return axis * slowPercent;
        }
        else
            return axis;
    }

    private double getVelX() {
        // Drive forward with negative Y on the joystick axis (forward)
        return applyGains(-kMaxSpeed * kTranslationXPort.getFilteredAxis(), kSlowModePercent);
    }
    private double getVelY() {
        // Drive left with negative X on the joystick axis (left)
        return applyGains(-kMaxSpeed * kTranslationYPort.getFilteredAxis(), kSlowModePercent);
    }
    private double getVelOmega() {
        // Drive counterclockwise with negative X on the joystick axis (left)
        return applyGains(kMaxAngularSpeed * -1.0 * kVelocityOmegaPort.getFilteredAxis(), kSlowModeRotationPercent);
    }
    
}
