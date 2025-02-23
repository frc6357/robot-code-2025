package frc.robot.bindings;

import static frc.robot.Konstants.ElevatorConstants.*;
import static frc.robot.Konstants.ElevatorConstants.ElevatorPosition.*;
import static frc.robot.Ports.OperatorPorts.*;

import java.util.Optional;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.ElevatorButtonCommand;
import frc.robot.commands.ElevatorJoystickCommand;
import frc.robot.subsystems.SK25Elevator;
import frc.robot.utils.filters.DeadbandFilter;

public class SK25ElevatorBinder implements CommandBinder{
    Optional<SK25Elevator> subsystem;

    Trigger LowButton;
    Trigger MidButton;
    Trigger TopButton;
    Trigger TroughButton;
    Trigger zeroPositionButton;
    //Trigger zeroPositionButtonDriver;
    Trigger resetPos;
    Trigger elevatorOverride;

    public SK25ElevatorBinder(Optional<SK25Elevator> subsystem){
        
        this.subsystem = subsystem;

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
        if (subsystem.isPresent())
        {
            SK25Elevator elevator = subsystem.get();

            double joystickGain = kJoystickReversed ? -kJoystickChange : kJoystickChange;
            kElevatorAxis.setFilter(new DeadbandFilter(kJoystickDeadband, joystickGain));

            // Elevator Position Buttons
            zeroPositionButton.onTrue(new ElevatorButtonCommand(kZeroPosition, elevator));
            TroughButton.onTrue(new ElevatorButtonCommand(kTroughPosition, elevator));
            LowButton.onTrue(new ElevatorButtonCommand(kLowPosition, elevator));
            MidButton.onTrue(new ElevatorButtonCommand(kMidPosition, elevator));
            TopButton.onTrue(new ElevatorButtonCommand(kTopPosition, elevator));

            elevatorOverride.whileTrue(new ElevatorJoystickCommand(
                () -> {return kElevatorAxis.getFilteredAxis();},
                () -> {return kElevatorOverride.button.getAsBoolean();},
                elevator));

            /* 
            elevator.setDefaultCommand(
                // Vertical movement of the elevator is controlled by the Y axis of the left stick.
                // Up on the joystick moves elevator up, and down on stick moves the elevator down.
                new ElevatorJoystickCommand(
                    () -> {return kElevatorAxis.getFilteredAxis();},
                    () -> {return kElevatorOverride.button.getAsBoolean();},
                    elevator));
            */
        }
    }
}