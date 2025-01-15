package frc.robot.bindings;

import static frc.robot.Konstants.ElevatorConstants.*;
import static frc.robot.Ports.OperatorPorts.kElevatorAxis;

import java.util.Optional;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.ElevatorCommand;
import frc.robot.subsystems.SK25Elevator;
import frc.robot.Ports;
import frc.robot.utils.filters.DeadbandFilter;

public class SK25ElevatorBinder implements CommandBinder{
    Optional<SK25Elevator> subsystem;
    Trigger elevatorUpDriverButton;
    Trigger elevatorDownDriverButton;
    Trigger rightElevatorButton;
    Trigger leftElevatorButton;
    Trigger elevatorUpOperatorButton;
    Trigger elevatorDownOperatorButton;
    Trigger elevatorOverride;


    public SK25ElevatorBinder(Optional<SK25Elevator> subsystem){
        
        this.subsystem = subsystem;
        /*
        this.climbUpDriverButton = Ports.DriverPorts.kClimbUp.button;
        this.climbDownDriverButton = Ports.DriverPorts.kClimbDown.button;
        this.climbUpOperatorButton = Ports.OperatorPorts.kClimbUp.button;
        this.climbDownOperatorButton = Ports.OperatorPorts.kClimbDown.button;
        this.climbOverride = Ports.DriverPorts.kClimbOverride.button;
        this.rightClimbButton = Ports.DriverPorts.kClimbRight.button;
        this.leftClimbButton = Ports.DriverPorts.kClimbLeft.button;
        */
    }

    public void bindButtons()
    {
        // If subsystem is present then this method will bind the buttons
        if (subsystem.isPresent())
        {
            SK25Elevator elevator = subsystem.get();

            double joystickGain = kJoystickReversed ? -kJoystickChange : kJoystickChange;
            kElevatorAxis.setFilter(new DeadbandFilter(kJoystickDeadband, joystickGain));

            // Elevator Up Buttons 
            elevatorUpDriverButton.or(elevatorUpOperatorButton).onTrue(new InstantCommand(() -> elevator.runRightMotor(kElevatorUpSpeed))); 
            elevatorUpDriverButton.or(elevatorUpOperatorButton).onTrue(new InstantCommand(() -> elevator.runLeftMotor(kElevatorUpSpeed))); 
            
            elevatorUpDriverButton.or(elevatorUpOperatorButton).onFalse(new InstantCommand(() -> elevator.runRightMotor(0.0))); 
            elevatorUpDriverButton.or(elevatorUpOperatorButton).onFalse(new InstantCommand(() -> elevator.runLeftMotor(0.0)));

            // Elevator Down Buttons
            elevatorDownDriverButton.or(elevatorDownOperatorButton).onTrue(new InstantCommand(() -> elevator.runRightMotor(kElevatorDownSpeed))); 
            elevatorDownDriverButton.or(elevatorDownOperatorButton).onTrue(new InstantCommand(() -> elevator.runLeftMotor(kElevatorDownSpeed))); 

            rightElevatorButton.onTrue(new InstantCommand(() -> elevator.runRightMotor(kElevatorDownSpeed)));
            leftElevatorButton.onTrue(new InstantCommand(() -> elevator.runLeftMotor(kElevatorDownSpeed)));
            
            elevatorDownDriverButton.or(elevatorDownOperatorButton).onFalse(new InstantCommand(() -> elevator.runLeftMotor(0.0)));
            elevatorDownDriverButton.or(elevatorDownOperatorButton).onFalse(new InstantCommand(() -> elevator.runRightMotor(0.0)));

            rightElevatorButton.onFalse(new InstantCommand(() -> elevator.runRightMotor(0.0)));
            leftElevatorButton.onFalse(new InstantCommand(() -> elevator.runLeftMotor(0.0)));
            
            elevatorUpDriverButton.and(elevatorOverride).onTrue(new InstantCommand(() -> elevator.resetPosition(1.0)));

              elevator.setDefaultCommand(
                         // Vertical movement of the arm is controlled by the Y axis of the right stick.
                         // Up on joystick moving arm up and down on stick moving arm down.
                         new ElevatorCommand(
                             () -> {return kElevatorAxis.getFilteredAxis();},
                             elevatorOverride::getAsBoolean,
                             elevator));
        }
    }
}