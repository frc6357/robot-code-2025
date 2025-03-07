package frc.robot.bindings;
import static frc.robot.Ports.OperatorPorts.kLowAlgae;
import static frc.robot.Ports.OperatorPorts.kElevatorOverride;
import static frc.robot.Ports.OperatorPorts.kHighAlgae;
import static frc.robot.Ports.OperatorPorts.kLowBranch;
import static frc.robot.Ports.OperatorPorts.kResetElevatorPos;
import static frc.robot.Ports.OperatorPorts.kTrough;
import static frc.robot.Ports.OperatorPorts.*;

import java.util.Optional;

import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.subsystems.CoralSubsystem;
import frc.robot.subsystems.CoralSubsystem.Setpoint;

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
    Trigger LowAlgae;
    Trigger HighAlgae;
    Trigger Net;

    public RevBindings(Optional<CoralSubsystem> elevatorSubsystem)
    {
        this.elevatorSubsystem  = elevatorSubsystem;
        this.elevatorOverride   = kElevatorOverride.button;
        this.zeroPositionButton = kZeroPositionOperator.button;
        this.LowButton          = kLowBranch.button;
        //this.MidButton          = kMiddleBranch.button;
        //this.TopButton          = kTopBranch.button;
        this.TroughButton       = kTrough.button;
        this.resetPos           = kResetElevatorPos.button;
        this.LowAlgae = kLowAlgae.button;
        this.HighAlgae = kHighAlgae.button;
        this.Net = kNetPos.button;
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
            //MidButton.onTrue(elevator.setSetpointCommand(Setpoint.kLevel3));
            //TopButton.onTrue(elevator.setSetpointCommand(Setpoint.kLevel4));
            LowAlgae.onTrue(elevator.setSetpointCommand(Setpoint.kLowAlgae));
            HighAlgae.onTrue(elevator.setSetpointCommand(Setpoint.kHighAlgae));
            Net.onTrue(elevator.setSetpointCommand(Setpoint.kNet));
        }
    }
}