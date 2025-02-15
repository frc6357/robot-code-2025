package frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.TempSwerve;

public class TempSwerveCommand extends Command{
    @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})

    //declare swerve object
    private final TempSwerve Swerve;

    //declare the double version of joystick input values.
    //Suppliers are used to update the value periodicaly. 
    private  Supplier<Double> leftY;
    private  Supplier<Double> leftX;
    private  Supplier<Double> rightX;

    /**
     * Creates a new SwerveCommand to handle all of the movement of the swerve drivebase.
     * @param _swerve The SK25Swerve object to house the movement methods for the drivebase.
     * @param _leftY The Y input of the left joystick on the controller used for translation.
     * @param _leftX The X input of the left joystick on the controller used for translation.
     * @param _rightX The X input of the right joystick on the controller used for rotation.
     */
    public TempSwerveCommand(TempSwerve _swerve, Supplier<Double> _leftY, Supplier<Double> _leftX, Supplier<Double> _rightX)
    {
        //initialize objects and variables
        Swerve = _swerve;
        leftY = _leftY;
        leftX = _leftX;
        rightX = _rightX;
        //ensure that this command only runs if no other command which uses the swerve subsystem is currently running.
        addRequirements(Swerve);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() 
    {
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() 
    {
        double desiredRadiansPerSecond = rightX.get() * 2 * Math.PI;

        //run the doSwerve method which handles all swerve movement possibilites.
        Swerve.normalDrive(leftX.get(), leftY.get(), desiredRadiansPerSecond);
    }
    

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) 
    {
       //sets all wheels to stop moving. //TODO: change to account for direction currently
       Swerve.normalDrive(0.0, 0.0, 0.0);
    }

    //no isFinished() method is present since this is the default command of the SK25Swerve subsystem.
}