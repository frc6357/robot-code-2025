package frc.robot.subsystems;

import static frc.robot.Konstants.ElevatorConstants.elevatorConfig;
import static frc.robot.Konstants.ElevatorConstants.kRightElevatorMotorID;

import java.util.function.Supplier;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Konstants.ElevatorConstants.ElevatorSetpoints;
import frc.robot.preferences.Pref;
import frc.robot.preferences.SKPreferences;

public class SK25Elevator extends SubsystemBase
{
    /** Elevator Setpoints */
    public enum ElevatorSetpoint
    {
        kZero,
        kTrough,
        kLevel2,
        kLevel3,
        kLevel4,
        kLowAlgae,
        kHighAlgae,
        kNet,
        kStation;
    }


  //Initialize Motor and controller with elevator configs in Konstants.
    private SparkFlex elevatorMotor = new SparkFlex(
        kRightElevatorMotorID,
        MotorType.kBrushless);
    private SparkClosedLoopController elevatorClosedLoopController = elevatorMotor.getClosedLoopController();
    private RelativeEncoder elevatorEncoder = elevatorMotor.getEncoder();

    //Set defualt limit switch values and elevator position to bottom position, else elevator will try to
    //reach position when robot is first enabled.
    private boolean wasResetByRioUserButton = false;
    // private boolean wasLimitSwitchPressed = false;
    private double elevatorCurrentHeight = 0.0;
    private double elevatorTargetHeight = ElevatorSetpoints.kZero;


    //Publisher for target elevator height
    final Pref<Double> elevatortargetHeight = SKPreferences.attach("elevatorTargetHeight", 0.0)
    .onChange((newValue) -> {elevatorTargetHeight = newValue;});
  
    //Publishers for PID tuning (P, I, D, velocity) by calling reconfigureElevator()
    final Pref<Double> elevatorKp = SKPreferences.attach("elevatorKp", 0.05)
    .onChange((newValue) -> reconfigureElevator());
    final Pref<Double> elevatorKi = SKPreferences.attach("elevatorKi", 0.0)
    .onChange((newValue) -> reconfigureElevator());
    final Pref<Double> elevatorKd = SKPreferences.attach("elevatorKd", 0.0008)
    .onChange((newValue) -> reconfigureElevator());
    final Pref<Double> elevatorVelocity = SKPreferences.attach("elevatorVelocity", 4000.0)
    .onChange((unused) -> reconfigureElevator());
  
    /** Applies the new configs from the publishers to the elevator config object. */
    private void reconfigureElevator()
    {
        elevatorConfig.closedLoop.pid(
            elevatorKp.get(),
            elevatorKi.get(), 
            elevatorKd.get());  
        elevatorConfig.closedLoop.maxMotion
            .maxVelocity(elevatorVelocity.get());      
        elevatorMotor.configure(
            elevatorConfig,
            ResetMode.kResetSafeParameters,
            PersistMode.kNoPersistParameters
        );
    }


    public SK25Elevator()
    {
        elevatorMotor.configure(
            elevatorConfig,
            //kResetSafeParameters is used to get the SPARK to a known configuratable state. This
            //is useful in case the SPARK is replaced, so the new motor has the same parameters.
            ResetMode.kResetSafeParameters,
            //kPersistParameters is used to ensure the configuration is not lost when
            //the SPARK loses power. This is useful for power cycles that may occur
            //mid-operation.
            PersistMode.kPersistParameters);

        // Zero elevator encoder when initialized
        elevatorEncoder.setPosition(0);
  }

  /** Moves the elevator to the target position by using the elevatorCurrentTarget class variable, the
   * applied SPARK/PID configs, and MAXMotion position enhancemtn, which uses trapezoid profiling to
   * control the max velocity and increase/decrease in acceleration of the motor to the target.*/
    private void moveToSetpoint()
    {
        elevatorClosedLoopController.setReference(
            elevatorTargetHeight,
            ControlType.kMAXMotionPositionControl);
    }

    /** Zero the elevator encoder when the limit switch is pressed. This is called periodicaly to constantly
    * update the wasResetByLimit Status.*/
    // private void zeroElevatorOnLimitSwitch() {
    //     if (!wasLimitSwitchPressed && elevatorMotor.getReverseLimitSwitch().isPressed())
    //     {
    //         // Zero the encoder only when the limit switch is switches from "unpressed" to "pressed" to
    //         // prevent constant zeroing while pressed
    //         elevatorEncoder.setPosition(0);
    //         wasLimitSwitchPressed = true;
    //     }
    //     else if (!elevatorMotor.getReverseLimitSwitch().isPressed())
    //     {
    //         wasLimitSwitchPressed = false;
    //     }
    // }

    /** Zero the arm and elevator encoders when the user button is pressed on the roboRIO. */
    private void zeroOnUserButton() {
        if (!wasResetByRioUserButton && RobotController.getUserButton())
        {
            // Zero the encoders only when button switches from "unpressed" to "pressed" to prevent
            // constant zeroing while pressed
            wasResetByRioUserButton = true;
            elevatorEncoder.setPosition(0);
        }
        else if (!RobotController.getUserButton())
        {
            wasResetByRioUserButton = false;
        }
    }

    /**Sets the setpoint (target elevator height) variable to the new height, denoted by the respective enum
    * constant. This method returns a command since I was too lazy (efficient) to write a new elevator button command
    * class.
    * @param setpoint The setpoint of the elevator in motor rotations.
    * @return The Command object which contains the (single run (runOnce()) command method, which runs 
    * its contents one time. In this case, the target elevator height is set, and the command ends imidiatley
    * after.*/
    public Command setSetpointCommand(ElevatorSetpoint setpoint)
    {
        return this.runOnce(() -> {
            switch (setpoint)
            {
                case kZero:
                    elevatorTargetHeight = ElevatorSetpoints.kZero;
                    break;
                case kTrough:
                    elevatorTargetHeight = ElevatorSetpoints.kTrough;
                    break;
                case kLevel2:
                    elevatorTargetHeight = ElevatorSetpoints.kLevel2;
                    break;
                case kLevel3:
                    elevatorTargetHeight = ElevatorSetpoints.kLevel3;
                    break;
                case kLevel4:
                    elevatorTargetHeight = ElevatorSetpoints.kLevel4;
                    break;
                case kLowAlgae:
                    elevatorTargetHeight = ElevatorSetpoints.kLowAlgae;
                    break;
                case kHighAlgae:
                    elevatorTargetHeight = ElevatorSetpoints.kHighAlgae;
                    break;
                case kNet:
                    elevatorTargetHeight = ElevatorSetpoints.kNet;
                    break;
                case kStation:
                    elevatorTargetHeight = ElevatorSetpoints.kIntake;
                    break;
            }
        });
    }

    /**For use with SwerveBinder acceleration limits on the swerve based on the elevator height.
    * Uses a supplier to get the most recent value.
    * @return The current elevator height supplier.
    */
    public Supplier<Double> getCurrentHeightMotorRotations()
    {
        Supplier<Double> currentHeightSupplier = ()-> elevatorCurrentHeight;
        return currentHeightSupplier;
    }

    @Override
    public void periodic()
    {
        elevatorCurrentHeight = elevatorEncoder.getPosition();

        moveToSetpoint();
        //zeroElevatorOnLimitSwitch();
        zeroOnUserButton();

        // Display subsystem values to Elastic
        SmartDashboard.putNumber("ElevatorTargetPos", elevatorTargetHeight);
        SmartDashboard.putNumber("ElevatorCurrentPos", elevatorCurrentHeight);
        // SmartDashboard.putBoolean("LimitSwitchPressed", wasLimitSwitchPressed);
    }

    public void testInit()
    {
    }
    
    public void testPeriodic()
    {
    }
}