package frc.robot.bindings;
import static frc.robot.Ports.OperatorPorts.*;

import java.util.Optional;

import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.subsystems.CoralSubsystem;
import frc.robot.subsystems.CoralSubsystem.Setpoint;

public class RevBindings implements CommandBinder
{
    Optional<CoralSubsystem> elevatorSubsystem;
    Trigger L2;
    Trigger L3;
    Trigger L4;
    Trigger Trough;
    Trigger zeroPosition;
    Trigger resetPos;
    Trigger elevatorOverride;
    Trigger LowAlgae;
    Trigger HighAlgae;
    Trigger Net;
    Trigger Station;

    public RevBindings(Optional<CoralSubsystem> elevatorSubsystem)
    {
        this.elevatorSubsystem = elevatorSubsystem;
        this.elevatorOverride = kElevatorOverride.button;
        this.zeroPosition = kZeroPos.button;
        this.L2 = kL2BranchPos.button;
        this.L3 = kL3BranchPos.button;
        this.L4 = kL4BranchPos.button;
        this.Trough = kTroughPos.button;
        this.resetPos = kResetElevatorPos.button;
        this.LowAlgae = kLowAlgaePos.button;
        this.HighAlgae = kHighAlgaePos.button;
        this.Net = kNetPos.button;
        this.Station = kStationPos.button;
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
            //              // Up on the joystick moves elevator up, and down on stick moves th++e elevator down.
            //              new ElevatorJoystickCommand(
            //                  () -> {return kElevatorAxis.getFilteredAxis();},
            //                  () -> {return kElevatorOverride.button.getAsBoolean();},
            //                  elevator));
            
            // Elevator Position Buttons
            zeroPosition.onTrue(elevator.setSetpointCommand(Setpoint.kZero));
            Trough.onTrue(elevator.setSetpointCommand(Setpoint.kTrough));
            L2.onTrue(elevator.setSetpointCommand(Setpoint.kLevel2));
            L3.onTrue(elevator.setSetpointCommand(Setpoint.kLevel3));
            L4.onTrue(elevator.setSetpointCommand(Setpoint.kLevel4));
            LowAlgae.onTrue(elevator.setSetpointCommand(Setpoint.kLowAlgae));
            HighAlgae.onTrue(elevator.setSetpointCommand(Setpoint.kHighAlgae));
            Net.onTrue(elevator.setSetpointCommand(Setpoint.kNet));
            Station.onTrue(elevator.setSetpointCommand(Setpoint.kIntake));
        }
    }
}