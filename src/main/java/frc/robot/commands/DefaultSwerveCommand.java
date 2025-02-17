// package frc.robot.commands;

// import java.util.function.BooleanSupplier;
// import java.util.function.DoubleSupplier;

// import edu.wpi.first.wpilibj2.command.Command;
// import frc.robot.subsystems.swerve.SK25Swerve;

// public class DefaultSwerveCommand extends Command {
//     private DoubleSupplier xSpeed;
//     private DoubleSupplier ySpeed;
//     private DoubleSupplier omegaSpeed;
//     private BooleanSupplier fieldCentric;

//     private SK25Swerve subsystem;

//     /**
//      * Creates a command that turns the robot to a specified angle using the field
//      * coordinate system
//      * 
//      * @param xSpeed
//      *            The supplier for the robot x axis speed
//      * @param ySpeed
//      *            The supplier for the robot y axis speed
//      * @param omegaSpeed
//      *            The supplier for the robot rotational speed
//      * @param robotCentric
//      *            Whether or not the drive mode is in robot or field centric mode
//      * @param drive
//      *            The subsystem required to control the drivetrain
//      */
//     public DefaultSwerveCommand(DoubleSupplier xSpeed, DoubleSupplier ySpeed, DoubleSupplier omegaSpeed,
//         BooleanSupplier fieldCentric, SK25Swerve drive)
//     {
//         this.xSpeed = xSpeed;
//         this.ySpeed = ySpeed;
//         this.omegaSpeed = omegaSpeed;

//         this.fieldCentric = fieldCentric;
//         this.subsystem = drive;
        
//         addRequirements(drive);
//     }

//     // Called every time the scheduler runs while the command is scheduled.
//     @Override
//     public void execute()
//     {
//         subsystem.drive(xSpeed.getAsDouble(), ySpeed.getAsDouble(), omegaSpeed.getAsDouble(), fieldCentric.getAsBoolean());
//     }

//     // Called once the command ends or is interrupted.
//     @Override
//     public void end(boolean interrupted)
//     {
//         subsystem.drive(
//             0.0, 0.0, 0.0,
//             true); // fieldRelative = true
//     }
// }
