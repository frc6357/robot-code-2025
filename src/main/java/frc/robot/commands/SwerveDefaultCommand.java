package frc.robot.commands;

import static frc.robot.Konstants.SwerveConstants.kJoystickDeadband;

import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.SK25SwerveFactory;

public class SwerveDefaultCommand extends Command{
    @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})

    //declare swerve object
    private final SK25SwerveFactory Swerve;

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
    public SwerveDefaultCommand(SK25SwerveFactory _swerve, Supplier<Double> _leftY, Supplier<Double> _leftX, Supplier<Double> _rightX)
    {
        //initialize objects and variables
        Swerve = _swerve;
        leftY = _leftY;
        leftX = _leftX;
        rightX = _rightX;
        //ensure that this command only runs if no other command which uses the swerve subsystem is currently running.
        addRequirements(Swerve);
    }

    //create a controller deadzone method which gives an output of zero if the controller is in the specified deadzone.
    private double deadZone (double value, double deadZone)
    {
        //if the joystick value is less than the deadzone  value
        if( Math.abs(value) < deadZone)
        {
            //override the value output as zero
            return 0;
        }
        //return the normal joystick output
        return value;
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
        
        //driverController conversion for left joystick.
        //converts Cartesian coordinate system to Polar system by getting the angle between left joystick X and Y components.
        //make the controller deadzones.
        double translationMagnitude = deadZone(Math.hypot(leftX.get(), leftY.get()), kJoystickDeadband);
        double xVelocity = deadZone(leftX.get(), kJoystickDeadband);
        double yVelocity = deadZone(leftY.get(), kJoystickDeadband);
        Translation2d velocity = new Translation2d(xVelocity, yVelocity).times(translationMagnitude);
        
        double rotationMagnitude = deadZone(rightX.get(), kJoystickDeadband);
        double desiredRadiansPerSecond = rotationMagnitude * 2 * Math.PI;

        //run the doSwerve method which handles all swerve movement possibilites.
        Swerve.doSwerve(desiredRadiansPerSecond, velocity);

        //SmartDashboard.putNumber("setpoint", Units.Radians.of(angle).in(Units.Degrees));
    }
    

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) 
    {
       //sets all wheels to stop moving. //TODO: change to account for direction currently
       Swerve.doSwerve(0.0, new Translation2d());
    }

    // Returns true when the command should end.
    //@Override
    //public boolean isFinished()
    //{
        //return false;
    //}
}
