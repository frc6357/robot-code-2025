package frc.robot.bindings;

import frc.robot.commands.EndEffectorButtonCommand;
import frc.robot.commands.EndEffectorJoystickCommand;

import static frc.robot.Konstants.EndEffectorConstants.kJoystickChange;
import static frc.robot.Konstants.EndEffectorConstants.kJoystickDeadband;
import static frc.robot.Konstants.EndEffectorConstants.kJoystickReversed;
import static frc.robot.Konstants.EndEffectorConstants.level1;
import static frc.robot.Konstants.EndEffectorConstants.level23;
import static frc.robot.Konstants.EndEffectorConstants.level4;
import static frc.robot.Konstants.EndEffectorConstants.intake;
import static frc.robot.Konstants.EndEffectorConstants.horizontal;
import static frc.robot.Ports.OperatorPorts.armHigh;
import static frc.robot.Ports.OperatorPorts.armMiddleLow;
import static frc.robot.Ports.OperatorPorts.armTrough;
import static frc.robot.Ports.OperatorPorts.intakebut;
import static frc.robot.Ports.OperatorPorts.zeropos;
import static frc.robot.Ports.OperatorPorts.endArm;


import java.util.Optional;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.subsystems.EndEffectorV1;
import frc.robot.utils.filters.DeadbandFilter;

public class EndEffectorBinder implements CommandBinder{
    Optional<EndEffectorV1> subsystem;
    

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

    Trigger zeroPositionButton;
    Trigger zeroPositionButtonDriver;

    Trigger resetPos;

    
    Trigger endEffectorAxis;


    public EndEffectorBinder(Optional<EndEffectorV1> subsystem){
        
        this.subsystem = subsystem;

        

        this.zeroPositionButton = zeropos.button;
        this.LowMidButton          = armMiddleLow.button;
        this.IntakeButton          = intakebut.button;
        this.TopButton          = armHigh.button;
        this.TroughButton       = armTrough.button;

    }

    public void bindButtons()
    {
        // If subsystem is present then this method will bind the buttons
        if (subsystem.isPresent())
        {
            EndEffectorV1 endEffector = subsystem.get();

           // double joystickGain = kJoystickReversed ? -kJoystickChange : kJoystickChange;
           // endArm.setFilter(new DeadbandFilter(kJoystickDeadband, joystickGain));
            System.out.println("Hi");

            // endEffector Up/Down Buttons 

        

            // endEffector Position Buttons (Kurian)

            zeroPositionButton.onTrue(new EndEffectorButtonCommand(horizontal, endEffector));
            TroughButton.onTrue(new EndEffectorButtonCommand(level1, endEffector));
            LowMidButton.onTrue(new EndEffectorButtonCommand(level23, endEffector));
            IntakeButton.onTrue(new EndEffectorButtonCommand(intake, endEffector));
            TopButton.onTrue(new EndEffectorButtonCommand(level4, endEffector));



           // endEffector.setDefaultCommand(
                    // Vertical movement of the arm is controlled by the Y axis of the right stick.
                    // Up on joystick moving arm up and down on stick moving arm down.
                   // new EndEffectorJoystickCommand(
                        //() -> {return endArm.getFilteredAxis();},
                       // endEffector));
                            
                             
        }
    }
}