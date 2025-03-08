package frc.robot.subsystems;

import static frc.robot.Konstants.ElevatorConstants.CoralSubsystemConstants.CoralSubsystem.elevatorConfig;
import static frc.robot.Konstants.ElevatorConstants.CoralSubsystemConstants.CoralSubsystem.kErrorTolerance;

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
import frc.robot.Konstants.ElevatorConstants.CoralSubsystemConstants;
import frc.robot.Konstants.ElevatorConstants.CoralSubsystemConstants.ElevatorSetpoints;
import frc.robot.preferences.Pref;
import frc.robot.preferences.SKPreferences;
import frc.robot.subsystems.SK25EndEffector;
import frc.robot.RobotContainer;

public class CoralSubsystem extends SubsystemBase {
  /** Subsystem-wide setpoints */
  public enum Setpoint {
    kZero,
    kLevel1,
    kLevel2,
    kLevel3,
    kLevel4,
    kLowAlgae,
    kHighAlgae,
    kNet,
    kIntake;
    //kProcessor;
  }


  // Initialize elevator SPARK. We will use MAXMotion position control for the elevator, so we also
  // need to initialize the closed loop controller and encoder.
  private SparkFlex elevatorMotor =
      new SparkFlex(CoralSubsystemConstants.kElevatorMotorCanId, MotorType.kBrushless);

  private SparkClosedLoopController elevatorClosedLoopController =
      elevatorMotor.getClosedLoopController();

  private RelativeEncoder elevatorEncoder = elevatorMotor.getEncoder();


  // Member variables for subsystem state management
  private boolean wasResetByButton = false;
  private boolean wasResetByLimit = false;
  private double elevatorCurrentTarget = ElevatorSetpoints.kZero;

  /* Suppliers for elevator velocity and position */
  public Supplier<Double> elevatorVelocity = (() -> (elevatorEncoder.getVelocity()));
  public Supplier<Double> elevatorPosition = (() -> (elevatorEncoder.getPosition()));

  

  /* Elevator PID preferences */

  final Pref<Double> elevatorKp = SKPreferences.attach("elevatorKp", 0.05)
    .onChange((newValue) -> {
        elevatorConfig.closedLoop.p(newValue);      
        elevatorMotor.configure(
          elevatorConfig,
          ResetMode.kResetSafeParameters,
          PersistMode.kPersistParameters
        );
    });

    final Pref<Double> elevatorKi = SKPreferences.attach("elevatorKi", 0.0)
    .onChange((newValue) -> {
        elevatorConfig.closedLoop.i(newValue);      
        elevatorMotor.configure(
          elevatorConfig,
          ResetMode.kResetSafeParameters,
          PersistMode.kPersistParameters
        );
    });

    final Pref<Double> elevatorKd = SKPreferences.attach("elevatorKd", 0.0025)
    .onChange((newValue) -> {
        elevatorConfig.closedLoop.d(newValue);      
        elevatorMotor.configure(
          elevatorConfig,
          ResetMode.kResetSafeParameters,
          PersistMode.kPersistParameters
        );
    });

    /* Elevator manual height adjusting preference */

    final Pref<Double> elevatortargetHeight = SKPreferences.attach("elevatorTargetHeight", 0.0)
    .onChange((newValue) -> {
        elevatorCurrentTarget = newValue;
    });



  /**
   * The main subsystem class used for moving coral up and down the elevator.
   * If looking for the scoring mechanism, see SK25EndEffector.
   */
  public CoralSubsystem() {
    /*
     * Apply the appropriate configurations to the SPARKs.
     *
     * kResetSafeParameters is used to get the SPARK to a known state. This
     * is useful in case the SPARK is replaced.
     *
     * kPersistParameters is used to ensure the configuration is not lost when
     * the SPARK loses power. This is useful for power cycles that may occur
     * mid-operation.
     */
    elevatorMotor.configure(
        elevatorConfig,
        ResetMode.kResetSafeParameters,
        PersistMode.kPersistParameters);

    // Zero arm and elevator encoders on initialization
    elevatorEncoder.setPosition(0);

  }

  /**
   * Drive the arm and elevator motors to their respective setpoints. This will use MAXMotion
   * position control which will allow for a smooth acceleration and deceleration to the mechanisms'
   * setpoints.
   */
  private void moveToSetpoint() {
    elevatorClosedLoopController.setReference(
        elevatorCurrentTarget, ControlType.kMAXMotionPositionControl);
  }

  /** Zero the elevator encoder when the limit switch is pressed. */
  private void zeroElevatorOnLimitSwitch() {
    if (!wasResetByLimit && elevatorMotor.getReverseLimitSwitch().isPressed()) {
      // Zero the encoder only when the limit switch is switches from "unpressed" to "pressed" to
      // prevent constant zeroing while pressed
      elevatorEncoder.setPosition(0);
      wasResetByLimit = true;
    } else if (!elevatorMotor.getReverseLimitSwitch().isPressed()) {
      wasResetByLimit = false;
    }
  }

  /** Zero the arm and elevator encoders when the user button is pressed on the roboRIO. */
  private void zeroOnUserButton() {
    if (!wasResetByButton && RobotController.getUserButton()) {
      // Zero the encoders only when button switches from "unpressed" to "pressed" to prevent
      // constant zeroing while pressed
      wasResetByButton = true;
      elevatorEncoder.setPosition(0);
    } else if (!RobotController.getUserButton()) {
      wasResetByButton = false;
    }
  }

  public boolean atSetpoint() {
      return (elevatorVelocity.get() < 0.1) 
      && (Math.abs(elevatorPosition.get() - elevatorCurrentTarget) < kErrorTolerance);
  }

  public boolean readyToScore(boolean ignoreLidar) {
    if(ignoreLidar) {
      return atSetpoint();
    }
    else {
      return atSetpoint(); //&& SK25EndEffector.haveCoral(); TODO: Get lidar hasCoral somehow
    }
  }



  /**
   * Command to set the subsystem setpoint. This will set the arm and elevator to their predefined
   * positions for the given setpoint.
   */
  public Command setSetpointCommand(Setpoint setpoint) {
    return this.runOnce(
        () -> {
          switch (setpoint) {
            case kZero:
              elevatorCurrentTarget = ElevatorSetpoints.kZero;
              break;
            case kLevel1:
              elevatorCurrentTarget = ElevatorSetpoints.kLevel1;
              break;
            case kLevel2:
              elevatorCurrentTarget = ElevatorSetpoints.kLevel2;
              break;
            case kLevel3:
              elevatorCurrentTarget = ElevatorSetpoints.kLevel3;
              break;
            case kLevel4:
              elevatorCurrentTarget = ElevatorSetpoints.kLevel4;
              break;
            case kLowAlgae:
              elevatorCurrentTarget = ElevatorSetpoints.kLowAlgae;
              break;
            case kHighAlgae:
              elevatorCurrentTarget = ElevatorSetpoints.kHighAlgae;
              break;
            case kNet:
              elevatorCurrentTarget = ElevatorSetpoints.kNet;
              break;
            case kIntake:
              elevatorCurrentTarget = ElevatorSetpoints.kIntake;
            // case kProcessor:
            //   elevatorCurrentTarget = ElevatorSetpoints.kProcessor;
            //   break;
          }
        });
  }

  @Override
  public void periodic() {
    moveToSetpoint();
    //zeroElevatorOnLimitSwitch();
    zeroOnUserButton();

    // Display subsystem values
    SmartDashboard.putNumber("Coral/Elevator/Velocity", elevatorVelocity.get());
    SmartDashboard.putNumber("Coral/Elevator/Target Position", elevatorCurrentTarget);
    SmartDashboard.putNumber("Coral/Elevator/Actual Position", elevatorEncoder.getPosition());

  }
}