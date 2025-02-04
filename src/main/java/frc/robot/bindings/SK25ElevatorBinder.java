package frc.robot.bindings;

import static frc.robot.Konstants.ElevatorConstants.kJoystickChange;
import static frc.robot.Konstants.ElevatorConstants.kJoystickDeadband;
import static frc.robot.Konstants.ElevatorConstants.kJoystickReversed;
import static frc.robot.Konstants.ElevatorConstants.ElevatorPosition.LowPosition;
import static frc.robot.Konstants.ElevatorConstants.ElevatorPosition.MidPosition;
import static frc.robot.Konstants.ElevatorConstants.ElevatorPosition.TopPosition;
import static frc.robot.Konstants.ElevatorConstants.ElevatorPosition.TroughPosition;
import static frc.robot.Konstants.ElevatorConstants.ElevatorPosition.ZeroPosition;
import static frc.robot.Ports.OperatorPorts.kElevatorAxis;
import static frc.robot.Ports.OperatorPorts.kLowBranch;
import static frc.robot.Ports.OperatorPorts.kMiddleBranch;
import static frc.robot.Ports.OperatorPorts.kResetElevatorPos;
import static frc.robot.Ports.OperatorPorts.kTopBranch;
import static frc.robot.Ports.OperatorPorts.kTrough;
import static frc.robot.Ports.OperatorPorts.kZeroPositionOperator;

import java.util.Optional;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.subsystems.SK25Elevator;
import frc.robot.utils.filters.DeadbandFilter;

public class SK25ElevatorBinder implements CommandBinder{
    Optional<SK25Elevator> subsystem;
    

    /* 
    Trigger rightElevatorButton;
    Trigger leftElevatorButton;
    Trigger elevatorUpOperatorButton;
    Trigger elevatorDownOperatorButton;
    */

    Trigger LowButton;
    Trigger MidButton;
    Trigger TopButton;
    Trigger TroughButton;

    Trigger zeroPositionButton;
    Trigger zeroPositionButtonDriver;

    Trigger resetPos;

    Trigger elevatorOverride;
    Trigger elevatorAxis;


    public SK25ElevatorBinder(Optional<SK25Elevator> subsystem){
        
        this.subsystem = subsystem;

        this.elevatorOverride = Ports.DriverPorts.kElevatorOverride.button;
        //this.elevatorAxis = Ports.DriverPorts.kElevatorAxis.button;

        this.zeroPositionButton = kZeroPositionOperator.button;
        this.LowButton          = kLowBranch.button;
        this.MidButton          = kMiddleBranch.button;
        this.TopButton          = kTopBranch.button;
        this.TroughButton       = kTrough.button;
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

        

            // Elevator Position Buttons (Kurian)

            zeroPositionButton.onTrue(new ElevatorButtonCommand(ZeroPosition, elevator));
            TroughButton.onTrue(new ElevatorButtonCommand(TroughPosition, elevator));
            LowButton.onTrue(new ElevatorButtonCommand(LowPosition, elevator));
            MidButton.onTrue(new ElevatorButtonCommand(MidPosition, elevator));
            TopButton.onTrue(new ElevatorButtonCommand(TopPosition, elevator));



              elevator.setDefaultCommand(
                         // Vertical movement of the arm is controlled by the Y axis of the right stick.
                         // Up on joystick moving arm up and down on stick moving arm down.
                         new ElevatorJoystickCommand(
                             () -> {return kElevatorAxis.getFilteredAxis();}))
                            
                             
        }
    }
}