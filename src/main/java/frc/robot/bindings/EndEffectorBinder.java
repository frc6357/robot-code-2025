package frc.robot.bindings;

import frc.robot.Konstants.EndEffectorConstants.EndEffectorPosition;
import frc.robot.commands.EndEffectorButtonCommand;
import frc.robot.commands.EndEffectorJoystickCommand;
import frc.robot.commands.EndEffectorRollerIntakeCommand;
import frc.robot.commands.EndEffectorRollerOutputCommand;
import frc.robot.commands.EndEffectorRollerStopCommand;
import frc.robot.commands.commandGroups.ScoreCommandGroup;
// import frc.robot.commands.EndEffectorEncoderResetCommand;
//import frc.robot.commands.EndEffectorStop;

//import static frc.robot.Konstants.EndEffectorConstants.kJoystickChange;
import static frc.robot.Konstants.EndEffectorConstants.kJoystickDeadband;
import static frc.robot.Konstants.EndEffectorConstants.kJoystickReversed;
import static frc.robot.Konstants.ElevatorConstants.ElevatorPosition.kLowPosition;
import static frc.robot.Konstants.ElevatorConstants.ElevatorPosition.kTopPosition;
import static frc.robot.Konstants.ElevatorConstants.ElevatorPosition.kTroughPosition;
import static frc.robot.Konstants.ElevatorConstants.ElevatorPosition.kZeroPosition;
import static frc.robot.Konstants.EndEffectorConstants.EndEffectorPosition.*;
import static frc.robot.Ports.OperatorPorts.armHigh;
import static frc.robot.Ports.OperatorPorts.armMiddleLow;
import static frc.robot.Ports.OperatorPorts.armTrough;
import static frc.robot.Ports.OperatorPorts.intakebut;
import static frc.robot.Ports.OperatorPorts.zeropos;
import static frc.robot.Ports.OperatorPorts.endArm;
import static frc.robot.Ports.OperatorPorts.resetencoder;
import static frc.robot.Ports.OperatorPorts.rollerintake;
import static frc.robot.Ports.OperatorPorts.rolleroutput;

import com.revrobotics.RelativeEncoder;

import java.util.Optional;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.subsystems.SK25EndEffector;
import frc.robot.subsystems.SK25Elevator;
import frc.robot.utils.filters.DeadbandFilter;

public class EndEffectorBinder implements CommandBinder {

    Optional<SK25EndEffector> endEffectorSubsystem;
    Optional<SK25Elevator> elevatorSubsystem;
    

    /* 
    Trigger rightendEffectorButton;
    Trigger leftendEffectorButton;
    Trigger endEffectorUpOperatorButton;
    Trigger endEffectorDownOperatorButton;
    */

    Trigger LowMidButton;
    Trigger TopButton;
    Trigger TroughButton;
    Trigger IntakeButton;
    Trigger ResetEncoderButton;
    Trigger RollerIntake;
    Trigger RollerOutPut;

    Trigger zeroPositionButton;
    Trigger zeroPositionButtonDriver;

    Trigger resetPos;

    
    Trigger endEffectorAxis;

    RelativeEncoder mEncoder;


    public EndEffectorBinder(Optional<SK25EndEffector> endEffectorSubsystem, Optional<SK25Elevator> elevatorSubsystem)
    {
        this.endEffectorSubsystem = endEffectorSubsystem;
        this.elevatorSubsystem = elevatorSubsystem;

        this.zeroPositionButton = zeropos.button;
        this.LowMidButton = armMiddleLow.button;
        this.IntakeButton = intakebut.button;
        this.TopButton = armHigh.button;
        this.TroughButton = armTrough.button;
        this.ResetEncoderButton = resetencoder.button;
        this.RollerIntake = rollerintake.button;
        this.RollerOutPut = rolleroutput.button;
    }

    public void bindButtons()
    {
        // If subsystem is present then this method will bind the buttons
        if (endEffectorSubsystem.isPresent() && elevatorSubsystem.isPresent())
        {
            SK25EndEffector endEffector = endEffectorSubsystem.get();
            SK25Elevator elevator = elevatorSubsystem.get();

            double joystickGain = kJoystickReversed ? -1 : 1;
            endArm.setFilter(new DeadbandFilter(kJoystickDeadband, joystickGain));

            // endEffector Position Buttons

            zeroPositionButton.onTrue(new ScoreCommandGroup(kZeroPosition, elevator, kZeroPositionAngle, endEffector));
            TroughButton.onTrue(new ScoreCommandGroup(kTroughPosition, elevator, kTroughPositionAngle, endEffector));
            LowMidButton.onTrue(new ScoreCommandGroup(kLowPosition, elevator, kMidLowPositionAngle, endEffector));
            IntakeButton.onTrue(new EndEffectorButtonCommand(kIntakePositionAngle, endEffector));
            TopButton.onTrue(new ScoreCommandGroup(kTopPosition, elevator, kTopPositionAngle, endEffector));
            
            // ResetEncoderButton.onTrue(new EndEffectorEncoderResetCommand(endEffector)); // TODO: Add back reset encoder command? This was originally for use with NEO Vortex pivot motor
            RollerIntake.onTrue(new EndEffectorRollerIntakeCommand(endEffector));
            RollerOutPut.onTrue(new EndEffectorRollerOutputCommand(endEffector));
            RollerIntake.onFalse(new EndEffectorRollerStopCommand(endEffector));
            RollerOutPut.onFalse(new EndEffectorRollerStopCommand(endEffector));

            endEffector.setDefaultCommand(
                    
                    // Vertical movement of the arm is controlled by the Y axis of the right stick.
                    // Up on joystick moving arm up and down on stick moving arm down.
                  new EndEffectorJoystickCommand(
                        () -> {return endArm.getFilteredAxis();},
                       endEffector));
                                      
        }
    }
}
    
