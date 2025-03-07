package frc.robot.bindings;
import java.util.Optional;

// Elevator subsystem
import frc.robot.subsystems.SK25Elevator;

// Constants for the elevator
import static frc.robot.Konstants.ElevatorConstants.*;

// Ports
import static frc.robot.Ports.OperatorPorts.*;

// Misc.
import edu.wpi.first.wpilibj2.command.button.Trigger;

// Commands
import frc.robot.commands.ElevatorJoystickCommand;
import frc.robot.utils.filters.DeadbandFilter;

// Unused imports
import frc.robot.commands.ElevatorButtonCommand;
import static frc.robot.Konstants.ElevatorConstants.ElevatorPosition.*;

public class SK25ElevatorBinder implements CommandBinder
{
    Optional<SK25Elevator> elevatorSubsystem;
    Trigger LowButton;
    Trigger MidButton;
    Trigger TopButton;
    Trigger TroughButton;
    Trigger zeroPositionButton;
    Trigger resetPos;
    Trigger elevatorOverride;
    Trigger NetButton;
    Trigger IntakeButton;

    public SK25ElevatorBinder(Optional<SK25Elevator> elevatorSubsystem)
    {
        this.elevatorSubsystem  = elevatorSubsystem;
        this.elevatorOverride   = kElevatorOverride.button;
        this.zeroPositionButton = kZeroPositionOperator.button;
        this.LowButton          = kLowBranch.button;
        this.MidButton          = kMiddleBranch.button;
        this.TopButton          = kTopBranch.button;
        this.TroughButton       = kTrough.button;
        this.resetPos           = kResetElevatorPos.button;
        this.NetButton          = kNetPos.button;
        this.IntakeButton       = kIntakePos.button;
    }

    public void bindButtons()
    {
        // If subsystem is present then this method will bind the buttons
        if (elevatorSubsystem.isPresent())
        {
            SK25Elevator elevator = elevatorSubsystem.get();

            double joystickGain = kJoystickReversed ? -kJoystickChange : kJoystickChange;
            kElevatorAxis.setFilter(new DeadbandFilter(kJoystickDeadband, joystickGain));

            elevatorOverride.whileTrue(new ElevatorJoystickCommand(
                () -> {return kElevatorAxis.getFilteredAxis();},
                () -> {return kElevatorOverride.button.getAsBoolean();},
                elevator));

            elevator.setDefaultCommand(
                         // Vertical movement of the elevator is controlled by the Y axis of the left stick.
                         // Up on the joystick moves elevator up, and down on stick moves the elevator down.
                         new ElevatorJoystickCommand(
                             () -> {return kElevatorAxis.getFilteredAxis();},
                             () -> {return kElevatorOverride.button.getAsBoolean();},
                             elevator));
            
            // Elevator Position Buttons
            zeroPositionButton.onTrue(new ElevatorButtonCommand(kZeroPosition, elevator));
            TroughButton.onTrue(new ElevatorButtonCommand(kTroughPosition, elevator));
            LowButton.onTrue(new ElevatorButtonCommand(kLowPosition, elevator));
            MidButton.onTrue(new ElevatorButtonCommand(kMidPosition, elevator));
            TopButton.onTrue(new ElevatorButtonCommand(kTopPosition, elevator));
            NetButton.onTrue(new ElevatorButtonCommand(kNetPosition, elevator));
            IntakeButton.onTrue(new ElevatorButtonCommand(kIntakePosition, elevator));
        }
    }
}