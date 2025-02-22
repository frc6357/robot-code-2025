package frc.robot.bindings;

import frc.robot.commands.EndEffectorButtonCommand;
import frc.robot.commands.EndEffectorJoystickCommand;
import frc.robot.commands.EndEffectorRollerIntakeCommand;
import frc.robot.commands.EndEffectorRollerOutputCommand;
import frc.robot.commands.EndEffectorRollerStopCommand;
import frc.robot.commands.EndEffectorEncoderResetCommand;
import frc.robot.commands.EndEffectorHoldCommand;
import frc.robot.commands.EndEffectorStop;

import static frc.robot.Konstants.EndEffectorConstants.kJoystickChange;
import static frc.robot.Konstants.EndEffectorConstants.kJoystickDeadband;
import static frc.robot.Konstants.EndEffectorConstants.kJoystickReversed;
import static frc.robot.Konstants.EndEffectorConstants.kLevel1Angle;
import static frc.robot.Konstants.EndEffectorConstants.kLevel23Angle;
import static frc.robot.Konstants.EndEffectorConstants.kLevel4Angle;
import static frc.robot.Konstants.EndEffectorConstants.kIntakeAngle;
import static frc.robot.Konstants.EndEffectorConstants.kArmTolerance;
import static frc.robot.Konstants.EndEffectorConstants.kHortizontalAngle;
import static frc.robot.Ports.OperatorPorts.armHigh;
import static frc.robot.Ports.OperatorPorts.armMiddleLow;
import static frc.robot.Ports.OperatorPorts.armTrough;
import static frc.robot.Ports.OperatorPorts.intakebut;
import static frc.robot.Ports.OperatorPorts.zeropos;
import static frc.robot.Ports.OperatorPorts.endArm;
import static frc.robot.Ports.OperatorPorts.resetencoder;
import static frc.robot.Ports.OperatorPorts.rollerintake;
import static frc.robot.Ports.OperatorPorts.rolleroutput;

import com.revrobotics.RelativeEncoder;

import java.util.Optional;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.subsystems.EndEffectorV2;
import frc.robot.utils.filters.DeadbandFilter;

public class EndEffectorBinder implements CommandBinder {

    Optional<EndEffectorV2> subsystem;
    

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
    Trigger ResetEncoderButton;
    Trigger RollerIntake;
    Trigger RollerOutPut;

    Trigger zeroPositionButton;
    Trigger zeroPositionButtonDriver;

    Trigger resetPos;

    
    Trigger endEffectorAxis;

    RelativeEncoder mEncoder;


    public EndEffectorBinder(Optional<EndEffectorV2> subsystem){
        
        this.subsystem = subsystem;

        

        this.zeroPositionButton = zeropos.button;
        this.LowMidButton = armMiddleLow.button;
        this.IntakeButton = intakebut.button;
        this.TopButton = armHigh.button;
        this.TroughButton = armTrough.button;
        this.ResetEncoderButton = resetencoder.button;
        this.RollerIntake = rollerintake.button;
        this.RollerOutPut = rolleroutput.button;
        


    }

    public void bindButtons()
    {
        // If subsystem is present then this method will bind the buttons
        if (subsystem.isPresent())
        {
            EndEffectorV2 endEffector = subsystem.get();

           double joystickGain = kJoystickReversed ? -kJoystickChange : kJoystickChange;
           endArm.setFilter(new DeadbandFilter(kJoystickDeadband, joystickGain));
            System.out.println("Hi");

            // endEffector Up/Down Buttons 

            

            


            // endEffector Position Buttons

            
            zeroPositionButton.onTrue(new EndEffectorButtonCommand(kHortizontalAngle, endEffector));
            TroughButton.onTrue(new EndEffectorButtonCommand(kLevel1Angle, endEffector));
            LowMidButton.onTrue(new EndEffectorButtonCommand(kLevel23Angle, endEffector));
            IntakeButton.onTrue(new EndEffectorButtonCommand(kIntakeAngle, endEffector));
            TopButton.onTrue(new EndEffectorButtonCommand(kLevel4Angle, endEffector));
            ResetEncoderButton.onTrue(new EndEffectorEncoderResetCommand(endEffector));
            RollerIntake.onTrue(new EndEffectorRollerIntakeCommand(endEffector));
            RollerOutPut.onTrue(new EndEffectorRollerOutputCommand(endEffector));
            RollerIntake.onFalse(new EndEffectorRollerStopCommand(endEffector));
            RollerOutPut.onFalse(new EndEffectorRollerStopCommand(endEffector));


            endEffector.setDefaultCommand(
                    
                    // Vertical movement of the arm is controlled by the Y axis of the right stick.
                    // Up on joystick moving arm up and down on stick moving arm down.
                    new EndEffectorJoystickCommand(
                        () -> {return endArm.getFilteredAxis();},
                       endEffector));
                             
        }
    }
}
    
