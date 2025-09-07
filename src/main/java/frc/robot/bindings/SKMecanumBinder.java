package frc.robot.bindings;

import static frc.robot.Ports.DriverPorts.kResetGyroPos;
import static frc.robot.Ports.DriverPorts.kRobotCentricMode;
import static frc.robot.Ports.DriverPorts.kSlowMode;

import java.util.Optional;

import edu.wpi.first.wpilibj.drive.MecanumDrive;
import edu.wpi.first.wpilibj2.command.button.Trigger;
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

    @Override
    public void bindButtons() {
        
    }
    
}
