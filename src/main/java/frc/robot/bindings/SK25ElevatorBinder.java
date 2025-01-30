package frc.robot.bindings;

import static frc.robot.Konstants.ElevatorConstants.*;
import static frc.robot.Ports.OperatorPorts.*;

import java.util.Optional;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.commands.ElevatorButtonCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.TroughCommand;
import frc.robot.commands.LowBranchCommand;
import frc.robot.commands.MiddleBranchCommand;
import frc.robot.commands.TopBranchCommand;
import frc.robot.commands.ElevatorJoystickCommand;
import frc.robot.subsystems.SK25Elevator;
import frc.robot.Ports;
import frc.robot.utils.filters.DeadbandFilter;

public class SK25ElevatorBinder implements CommandBinder{
    Optional<SK25Elevator> subsystem;
    Trigger elevatorTrough;
    Trigger elevatorLowBranch;
    Trigger elevatorMiddleBranch;
    Trigger elevatorTopBranch;
    /* 
    Trigger rightElevatorButton;
    Trigger leftElevatorButton;
    Trigger elevatorUpOperatorButton;
    Trigger elevatorDownOperatorButton;
    */

    Trigger LowButton;
    Trigger MidButton;
    Trigger HighButton;
    Trigger TroughButton;

    Trigger zeroPositionButton;
    Trigger zeroPositionButtonDriver;

    Trigger resetPos;

    Trigger elevatorOverride;
    Trigger elevatorAxis;


    public SK25ElevatorBinder(Optional<SK25Elevator> subsystem){
        
        this.subsystem = subsystem;
        
        this.elevatorTrough = Ports.OperatorPorts.kTrough.button;
        this.elevatorLowBranch = Ports.OperatorPorts.kLowBranch.button;
        this.elevatorMiddleBranch = Ports.OperatorPorts.kMiddleBranch.button;
        this.elevatorTopBranch = Ports.OperatorPorts.kTopBranch.button;

        this.elevatorOverride = Ports.DriverPorts.kElevatorOverride.button;
        //this.elevatorAxis = Ports.DriverPorts.kElevatorAxis.button;

        this.zeroPositionButton = kZeroPositionOperator.button;
        this.LowButton          = kLowBranch.button;
        this.MidButton          = kMiddleBranch.button;
        this.HighButton         = kTopBranch.button;
        this.TroughButton       = kTrough.button;
        this.resetPos           = kResetArmPos.button;
    }

    public void bindButtons()
    {
        // If subsystem is present then this method will bind the buttons
        if (subsystem.isPresent())
        {
            SK25Elevator elevator = subsystem.get();

            double joystickGain = kJoystickReversed ? -kJoystickChange : kJoystickChange;
            kElevatorAxis.setFilter(new DeadbandFilter(kJoystickDeadband, joystickGain));

            // Elevator Up/Down Buttons 

            elevatorTrough.onTrue(new TroughCommand(elevator));
            elevatorLowBranch.onTrue(new LowBranchCommand(elevator));
            elevatorMiddleBranch.onTrue(new MiddleBranchCommand(elevator));
            elevatorTopBranch.onTrue(new TopBranchCommand(elevator));

              elevator.setDefaultCommand(
                         // Vertical movement of the arm is controlled by the Y axis of the right stick.
                         // Up on joystick moving arm up and down on stick moving arm down.
                         new ElevatorJoystickCommand(
                             () -> {return kElevatorAxis.getFilteredAxis();},
                             elevatorOverride::getAsBoolean,
                             elevator));
        }
    }
}