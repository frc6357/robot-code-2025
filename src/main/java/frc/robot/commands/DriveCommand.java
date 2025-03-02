package frc.robot.commands;

import java.util.function.Supplier;

import frc.robot.RobotContainer;
import frc.robot.subsystems.SKSwerve;

import com.ctre.phoenix6.swerve.SwerveRequest;
import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;

import edu.wpi.first.wpilibj2.command.Command;

public class DriveCommand {
    private SKSwerve m_swerve = RobotContainer.m_swerve;

    Supplier<Double> velX;
    Supplier<Double> velY;
    Supplier<Double> rotRate;
    Supplier<Boolean> fieldOriented;

    
    
        private static final SwerveRequest.FieldCentric fieldCentricDrive = new SwerveRequest.FieldCentric()
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage);
        private static final SwerveRequest.RobotCentric robotCentricDrive = new SwerveRequest.RobotCentric()
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage);
    
        /**
         * This feeds raw velocities into the swerve drive for use by subsystems using their own velocity controllers
         * @param velX The translational X velocity of the swerve drive
         * @param velY The translational Y velocity of the swerve drive
         * @param rotRate The rotational velocity of the swerve drive
         * @param fieldOriented Whether or not the drive type should be field centric
         * @return Drive command that runs the swerve chassis
         */
        public DriveCommand(
                Supplier<Double> velX,
                Supplier<Double> velY,
                Supplier<Double> rotRate,
                Supplier<Boolean> fieldOriented) 
        {
            this.velX = velX;
            this.velY = velY;
            this.rotRate = rotRate;
            this.fieldOriented = fieldOriented;
        }
        
        public void run(){
            if(fieldOriented.get() == true) { // Field centric drive requested
                m_swerve.applyRequest(() -> {
                    return fieldCentricDrive
                        .withVelocityX(velX.get())
                        .withVelocityY(velY.get())
                        .withRotationalRate(rotRate.get());
                    }
                );
            }
            else { // Robot centric drive
                m_swerve.applyRequest(() -> {
                    return robotCentricDrive
                        .withVelocityX(velX.get())
                        .withVelocityY(velY.get())
                        .withRotationalRate(rotRate.get());
                    }
                );
            }
        }
}
