package frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import frc.robot.subsystems.SKMecanumDrive;


/**
 * This isn't actually considered a command. Rather, it's a simplified method of feeding raw 
 * velocities into the swerve drive.
 * USE THIS WITH CAUTION! It has no filter applied and if sudden and large velocity changes
 * are used, brownouts will likely occur. Try using a slew filter on the velocities passed into
 * this DriveCommand in order to limit their maximum rate of change. (Limiting acceleration)
 */
public class DriveCommand {
    private SKMecanumDrive m_drivetrain;

    Supplier<Double> velX;
    Supplier<Double> velY;
    Supplier<Double> rotRate;
    Supplier<Boolean> fieldOriented;
    
        /**
         * This feeds raw velocities into the drivetrain for use by subsystems using their own velocity controllers
         * @param velX The translational X velocity of the swerve drive
         * @param velY The translational Y velocity of the swerve drive
         * @param rotRate The rotational velocity of the swerve drive (radians/sec)
         * @param fieldOriented Whether or not the drive type should be field centric
         * @return An object to call run() on that applies raw velocities to the swerve chassis
         */
        public DriveCommand(
                SKMecanumDrive m_drivetrain,
                Supplier<Double> velX,
                Supplier<Double> velY,
                Supplier<Double> rotRate,
                Supplier<Boolean> fieldOriented) 
        {
            this.m_drivetrain = m_drivetrain;
            this.velX = velX;
            this.velY = velY;
            this.rotRate = rotRate;
            this.fieldOriented = fieldOriented;
        }
        
        public void run(){
            ChassisSpeeds speeds = new ChassisSpeeds(
                velX.get(), velY.get(), rotRate.get());
            
            if(fieldOriented.get() == true) { // Field centric drive requested
                // Since field-centric was requested, we assume the inputted velocities are field-centric
                m_drivetrain.setControl(
                    ChassisSpeeds.fromFieldRelativeSpeeds(speeds, m_drivetrain.getIMURotation())
                );
            }
            else { // Robot centric drive
                m_drivetrain.setControl(
                    speeds
                );
            }
        }
}
