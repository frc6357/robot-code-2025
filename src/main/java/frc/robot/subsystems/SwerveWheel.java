package frc.robot.subsystems;

import static frc.robot.Konstants.SwerveConstants.kPIDControllerTolerance;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.hardware.core.CoreCANcoder;
import com.ctre.phoenix6.swerve.utility.PhoenixPIDController;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class SwerveWheel {

    //declare variables for constructor
    PhoenixPIDController directionController;
    TalonFX driveMotor;
    TalonFX turnMotor;
    CoreCANcoder encoder;
    Angle offset;
    BaseStatusSignal status;
    double inverted;
    PIDController pcon;

    //declare and intitialize the encoder distance variables to zero before further manipulation.
    double currentDistance = 0.0;
    double targetDistance = 0.0;

    /**
     * Creates a new SwerveWheel which provides methods to control the drive and turn motors of the robot.
     * @param P The P value in the turning motor's PID controller.
     * @param I The I value in the turning motor's PID controller.
     * @param D The D value in the turning motor's PID controller. 
     * @param driveID The ID of the drive motor to be used with this SwerveWheel.
     * @param turnID The ID of the turn motor to be used with this SwerveWheel.
     * @param encoderID The ID of the encoder to be used with this SwerveWheel.
     * @param offset The offset constant of the encoder.
     * @param inverted If the motor is inverted. -1.0 if true, 1.0 if false.
     */
    public SwerveWheel(double P, double I, double D, int driveID, int turnID, int encoderID, Angle offset, double inverted)
    {
        //initialize constructor variables
        directionController = new PhoenixPIDController(P, I, D);
        driveMotor = new TalonFX(driveID,"SwerveCANivore"); //TODO: put inconstants
        turnMotor = new TalonFX(turnID, "SwerveCANivore");
        encoder = new CoreCANcoder(encoderID, "SwerveCANivore");
        this.offset = offset;
        this.inverted = inverted;
        pcon = new PIDController(0.0, 0.0, 0.0);
        
        //reset the PID controller once when the code is deployed.
        directionController.reset();
    }

    /**
     * Sets the direction of the wheels by setting the turn motor to the calculated difference between the current and target pose of the turn motor.
     * @param setpoint The target position of the turning motor to reach as a percentage value between 0.0 and 1.0.
     */
    public void setDirection(Angle setpoint)
    {
        //allows the PID loop to take the smaller of the two errors, for example, traveling -90 degrees instead of 270.
        directionController.enableContinuousInput(-180, 180);
        //sets the acceptable error bound to which the controller will stop if reached
        directionController.setTolerance(kPIDControllerTolerance);

        //get the absolute pos of the encoder as type StatusSingal<Angle>.
        StatusSignal<Angle> encoderAngle = encoder.getAbsolutePosition();
        //Subtract the encoder offset with the StatusSingal<Angle> type using the getValue() method to convert to the Angle type.
        Angle encoderPos = encoderAngle.getValue().minus(offset);
        //converts the Angle type to a primative double of the respective angle. 
        Double encoderDoublePos = encoderPos.in(Units.Degrees);
        //rotations from angle
        //Rotation encoderRotations = encoderPos.in(Units.Rotation);

        //TODO: remove later
        //SmartDashboard.putNumber("setpoint", setpoint.inDegrees);
        
        //set the motor to the target PID condition. Account for inversion using the inversion parameter.
        //Phoenix6 PID gains
        Slot0Configs m_PID = new Slot0Configs();
        m_PID.kP = 6.0;
        m_PID.kI = 0.0;
        m_PID.kD = 0.0; 
        turnMotor.getConfigurator().apply(m_PID);

        //pcon.setSetpoint(setpoint.in(Units.Degrees));

        //turnMotor.setPosition(inverted * pcon.calculate(encoderDoublePos, setpoint.in(Units.Degrees)));

        // create a position closed-loop request, voltage output, slot 0 configs
        final PositionVoltage m_request = new PositionVoltage(setpoint).withSlot(0);

        // set position to 10 rotations
        turnMotor.setControl(m_request);



        //if the PID controller is at the setpoint or within the error tolerance.
        if(directionController.atSetpoint())
        {
            //reset the PID controller after it is at the setpoint.
            directionController.reset();
        }
    }

    
    /**
     * Sets the speed of the drive motor.
     * @param speed The speed of the motor as a percentage between 0.0 and 1.0.
     */
    public void setSpeed(double speed)
    {
        driveMotor.set(speed);
    }
}
