package frc.robot.bindings;

import static frc.robot.Konstants.EndEffectorConstants.kJoystickDeadband;
import static frc.robot.Konstants.EndEffectorConstants.kJoystickReversed;
import static frc.robot.Konstants.EndEffectorConstants.EndEffectorPosition.kIntake;
import static frc.robot.Ports.OperatorPorts.*;

import java.util.Optional;

import com.revrobotics.RelativeEncoder;

import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Konstants.EndEffectorConstants.EndEffectorPosition;
import frc.robot.commands.EndEffectorButtonCommand;
import frc.robot.commands.EndEffectorEncoderResetCommand;
import frc.robot.commands.EndEffectorJoystickCommand;
import frc.robot.commands.EndEffectorRollerIntakeCommand;
import frc.robot.commands.EndEffectorRollerOutputCommand;
import frc.robot.commands.EndEffectorRollerStopCommand;
import frc.robot.subsystems.SK25EndEffector;
import frc.robot.utils.konstantLib.filters.DeadbandFilter;

public class SK25EndEffectorBinder implements CommandBinder {

    Optional<SK25EndEffector> endEffectorSubsystem;

    Trigger LowButton;
    Trigger MiddleButton;
    Trigger TopButton;
    Trigger TroughButton;
    Trigger StationButton;
    Trigger ResetEncoderButton;
    Trigger RollerIntake;
    Trigger RollerOutPut;
    Trigger zeroPositionButton;
    Trigger zeroPositionButtonDriver;
    Trigger resetPos;
    Trigger endEffectorAxis;
    Trigger NetButton;
    Trigger HighAlgaeButton;
    Trigger LowAlgae;
    Trigger Processor;

    RelativeEncoder mEncoder;

    public SK25EndEffectorBinder(Optional<SK25EndEffector> endEffectorSubsystem)
    {
        this.endEffectorSubsystem = endEffectorSubsystem;
        this.ResetEncoderButton = resetEndEffectorEncoder.button;
        this.RollerIntake = kIntake.button;
        this.RollerOutPut = kExtake.button;
        this.zeroPositionButton = kZeroPos.button;
        this.LowButton = kL2BranchPos.button;
        this.MiddleButton = kL3BranchPos.button;
        this.StationButton = kStationPos.button;
        this.TopButton = kL4BranchPos.button;
        this.TroughButton = kTroughPos.button;
        this.LowAlgae = kLowAlgaePos.button;
        this.HighAlgaeButton = kHighAlgaePos.button;
        this.NetButton = kNetPos.button;
    }

    public void bindButtons()
    {
        // If subsystem is present then this method will bind the buttons
        if (endEffectorSubsystem.isPresent())
        {
            SK25EndEffector endEffector = endEffectorSubsystem.get();

            //if the joystick is reversed, reverse the axis inputs
            double joystickGain = kJoystickReversed ? -1 : 1;
            //apply the deaband filter to the controller
            kEndEffectorAxis.setFilter(new DeadbandFilter(kJoystickDeadband, joystickGain));

            //The joysitck commmand is default, since having the manual control option for a subsystem
            //overide the automated ones (button commands), allows for easier debugging.
            endEffector.setDefaultCommand(
                    // Vertical movement of the arm is controlled by the Y axis of the right stick.
                    // Up on joystick moving arm up and down on stick moving arm down.
                  new EndEffectorJoystickCommand(
                        () -> {return kEndEffectorAxis.getFilteredAxis();},
                       endEffector));
            

            //Reset Encoder
            ResetEncoderButton.onTrue(new EndEffectorEncoderResetCommand(endEffector));

            //Coral
            zeroPositionButton.onTrue(new EndEffectorButtonCommand(EndEffectorPosition.kZeroPositionAngle, endEffector));
            TroughButton.onTrue(new EndEffectorButtonCommand(EndEffectorPosition.kTroughPositionAngle, endEffector));
            LowButton.onTrue(new EndEffectorButtonCommand(EndEffectorPosition.kLowPositionAngle, endEffector));
            MiddleButton.onTrue(new EndEffectorButtonCommand(EndEffectorPosition.kMiddleAngle, endEffector));
            StationButton.onTrue(new EndEffectorButtonCommand(EndEffectorPosition.kIntakePositionAngle, endEffector));
            TopButton.onTrue(new EndEffectorButtonCommand(EndEffectorPosition.kTopPositionAngle, endEffector));   
            
            //Algae
            NetButton.onTrue(new EndEffectorButtonCommand(EndEffectorPosition.kNetAngle, endEffector));
            HighAlgaeButton.onTrue(new EndEffectorButtonCommand(EndEffectorPosition.kHighAlgae, endEffector));
            LowAlgae.onTrue(new EndEffectorButtonCommand(EndEffectorPosition.kLowAlgae, endEffector)); 
            
            //Rollers
            RollerIntake.onTrue(new EndEffectorRollerIntakeCommand(endEffector));
            RollerOutPut.onTrue(new EndEffectorRollerOutputCommand(endEffector));
            RollerIntake.onFalse(new EndEffectorRollerStopCommand(endEffector));
            RollerOutPut.onFalse(new EndEffectorRollerStopCommand(endEffector));   
        }
    }
}
    
