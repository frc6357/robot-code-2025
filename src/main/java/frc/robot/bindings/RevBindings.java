package frc.robot.bindings;
import java.util.Optional;


// Constants for the elevator
import static frc.robot.Konstants.ElevatorConstants.*;

// Ports
import static frc.robot.Ports.OperatorPorts.*;

// Misc.
import edu.wpi.first.wpilibj2.command.button.Trigger;

// Commands
import frc.robot.commands.ElevatorJoystickCommand;
import frc.robot.subsystems.CoralSubsystem;
import frc.robot.utils.filters.DeadbandFilter;
import frc.robot.subsystems.CoralSubsystem.Setpoint;

// Unused imports
import frc.robot.commands.ElevatorButtonCommand;

public class RevBindings implements CommandBinder
{
    Optional<CoralSubsystem> elevatorSubsystem;
    Trigger LowButton;
    Trigger MidButton;
    Trigger TopButton;
    Trigger TroughButton;
    Trigger zeroPositionButton;
    Trigger resetPos;
    Trigger elevatorOverride;

    public RevBindings(Optional<CoralSubsystem> elevatorSubsystem)
    {
        this.elevatorSubsystem  = elevatorSubsystem;
        this.elevatorOverride   = kElevatorOverride.button;
        this.zeroPositionButton = kZeroPositionOperator.button;
        this.LowButton          = kLowBranch.button;
        this.MidButton          = kMiddleBranch.button;
        this.TopButton          = kTopBranch.button;
        this.TroughButton       = kTrough.button;
        this.resetPos           = kResetElevatorPos.button;
    }

    public void bindButtons()
    {
        // If subsystem is present then this method will bind the buttons
        if (elevatorSubsystem.isPresent())
        {
            CoralSubsystem elevator = elevatorSubsystem.get();

            // double joystickGain = kJoystickReversed ? -kJoystickChange : kJoystickChange;
            // kElevatorAxis.setFilter(new DeadbandFilter(kJoystickDeadband, joystickGain));

            // elevatorOverride.whileTrue(new ElevatorJoystickCommand(
            //     () -> {return kElevatorAxis.getFilteredAxis();},
            //     () -> {return kElevatorOverride.button.getAsBoolean();},
            //     elevator));

            // elevator.setDefaultCommand(
            //              // Vertical movement of the elevator is controlled by the Y axis of the left stick.
            //              // Up on the joystick moves elevator up, and down on stick moves the elevator down.
            //              new ElevatorJoystickCommand(
            //                  () -> {return kElevatorAxis.getFilteredAxis();},
            //                  () -> {return kElevatorOverride.button.getAsBoolean();},
            //                  elevator));
            
            // Elevator Position Buttons
            zeroPositionButton.onTrue(elevator.setSetpointCommand(Setpoint.kZero));
            TroughButton.onTrue(elevator.setSetpointCommand(Setpoint.kLevel1));
            LowButton.onTrue(elevator.setSetpointCommand(Setpoint.kLevel2));
            MidButton.onTrue(elevator.setSetpointCommand(Setpoint.kLevel3));
            TopButton.onTrue(elevator.setSetpointCommand(Setpoint.kLevel4));
        }
    }
}