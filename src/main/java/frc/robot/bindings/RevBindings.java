package frc.robot.bindings;
import static frc.robot.Konstants.EndEffectorConstants.kJoystickReversed;
import static frc.robot.Konstants.OIConstants.kJoystickDeadband;
import static frc.robot.Ports.OperatorPorts.kElevatorAxis;
//import static frc.robot.Ports.OperatorPorts.kLowBranch;
//import static frc.robot.Ports.OperatorPorts.kTrough;
import static frc.robot.Ports.OperatorPorts.kElevatorOverride;
import static frc.robot.Ports.OperatorPorts.kEndEffectorAxis;
import static frc.robot.Ports.OperatorPorts.kHighAlgae;
import static frc.robot.Ports.OperatorPorts.kIntakePos;
import static frc.robot.Ports.OperatorPorts.kLowAlgae;
import static frc.robot.Ports.OperatorPorts.kLowBranchEffector;
import static frc.robot.Ports.OperatorPorts.kMiddleBranchEffector;
import static frc.robot.Ports.OperatorPorts.kNetPos;
import static frc.robot.Ports.OperatorPorts.kResetElevatorPos;
import static frc.robot.Ports.OperatorPorts.kTopBranchEffector;
import static frc.robot.Ports.OperatorPorts.kTroughEffector;
import static frc.robot.Ports.OperatorPorts.kZeroPositionOperator;

import java.util.Optional;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.CoralElevatorJoystickCommand;
import frc.robot.commands.EndEffectorJoystickCommand;
import frc.robot.subsystems.CoralSubsystem;
import frc.robot.subsystems.CoralSubsystem.Setpoint;
import frc.robot.utils.filters.DeadbandFilter;

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
    Trigger Intake;

    public RevBindings(Optional<CoralSubsystem> elevatorSubsystem)
    {
        this.elevatorSubsystem  = elevatorSubsystem;
        this.elevatorOverride   = kElevatorOverride.button;
        this.zeroPositionButton = kZeroPositionOperator.button;
        this.LowButton          = kLowBranchEffector.button;
        this.MidButton          = kMiddleBranchEffector.button;
        this.TopButton          = kTopBranchEffector.button;
        this.TroughButton       = kTroughEffector.button;
        this.resetPos           = kResetElevatorPos.button;
        this.LowAlgae = kLowAlgae.button;
        this.HighAlgae = kHighAlgae.button;
        this.Net = kNetPos.button;
        this.Intake = kIntakePos.button;
    }

    public void bindButtons()
    {
        // If subsystem is present then this method will bind the buttons
        if (elevatorSubsystem.isPresent())
        {
            CoralSubsystem elevator = elevatorSubsystem.get();


            // double joystickGain = kJoystickReversed ? -1 : 1;
            // kElevatorAxis.setFilter(new DeadbandFilter(kJoystickDeadband, joystickGain));

            // elevator.setDefaultCommand(
            //     new CoralElevatorJoystickCommand(() -> {return kElevatorAxis.getFilteredAxis();}, elevator));
                
            // Elevator Position Buttons
            zeroPositionButton.onTrue(elevator.setSetpointCommand(Setpoint.kZero));
            TroughButton.onTrue(elevator.setSetpointCommand(Setpoint.kLevel1));
            LowButton.onTrue(elevator.setSetpointCommand(Setpoint.kLevel2));
            MidButton.onTrue(elevator.setSetpointCommand(Setpoint.kLevel3));
            TopButton.onTrue(elevator.setSetpointCommand(Setpoint.kLevel4));
            LowAlgae.onTrue(elevator.setSetpointCommand(Setpoint.kLowAlgae));
            HighAlgae.onTrue(elevator.setSetpointCommand(Setpoint.kHighAlgae));
            Net.onTrue(elevator.setSetpointCommand(Setpoint.kLevel4));
            Intake.onTrue(elevator.setSetpointCommand(Setpoint.kIntake));
        }
    }
}