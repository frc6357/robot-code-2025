package frc.robot.bindings;

import static frc.robot.Konstants.EndEffectorConstants.kJoystickDeadband;
import static frc.robot.Konstants.EndEffectorConstants.kJoystickReversed;
import static frc.robot.Konstants.EndEffectorConstants.kRollerIntakeSpeed;
import static frc.robot.Konstants.EndEffectorConstants.kRollerEjectSpeed;
import static frc.robot.Ports.OperatorPorts.kEndEffectorAxis;
import static frc.robot.Ports.OperatorPorts.kEndEffectorEncoderReset;
import static frc.robot.Ports.OperatorPorts.kExtake;
import static frc.robot.Ports.OperatorPorts.kHighAlgaePos;
import static frc.robot.Ports.OperatorPorts.kIntake;
import static frc.robot.Ports.OperatorPorts.kL2BranchPos;
import static frc.robot.Ports.OperatorPorts.kL3BranchPos;
import static frc.robot.Ports.OperatorPorts.kL4BranchPos;
import static frc.robot.Ports.OperatorPorts.kLowAlgaePos;
import static frc.robot.Ports.OperatorPorts.kNetPos;
import static frc.robot.Ports.OperatorPorts.kStationPos;
import static frc.robot.Ports.OperatorPorts.kTroughPos;
import static frc.robot.Ports.OperatorPorts.kZeroPos;

import java.util.Optional;

import com.revrobotics.RelativeEncoder;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Konstants.EndEffectorConstants.EndEffectorPosition;
import frc.robot.commands.EndEffectorButtonCommand;
import frc.robot.commands.EndEffectorJoystickCommand;
//import frc.robot.commands.EndEffectorRollerIntakeCommand;
//import frc.robot.commands.EndEffectorRollerOutputCommand;
//import frc.robot.commands.EndEffectorRollerStopCommand;
import frc.robot.subsystems.SK25EndEffector;
import frc.robot.utils.konstantLib.filters.DeadbandFilter;

public class SK25EndEffectorBinder implements CommandBinder {

    Optional<SK25EndEffector> endEffectorSubsystem;

    Trigger L2Button;
    Trigger L3Button;
    Trigger L4Button;
    Trigger TroughButton;
    Trigger StationButton;
    Trigger ResetEncoderButton;
    Trigger RollerIntake;
    Trigger RollerExtake;
    Trigger ZeroPositionButton;
    Trigger ResetPos;
    Trigger EndEffectorAxis;
    Trigger NetButton;
    Trigger HighAlgaeButton;
    Trigger LowAlgaeButton;

    RelativeEncoder mEncoder;

    public SK25EndEffectorBinder(Optional<SK25EndEffector> endEffectorSubsystem)
    {
        this.endEffectorSubsystem = endEffectorSubsystem;
        this.ResetEncoderButton = kEndEffectorEncoderReset.button;
        this.RollerIntake = kIntake.button;
        this.RollerExtake = kExtake.button;
        this.ZeroPositionButton = kZeroPos.button;
        this.L2Button = kL2BranchPos.button;
        this.L3Button = kL3BranchPos.button;
        this.StationButton = kStationPos.button;
        this.L4Button = kL4BranchPos.button;
        this.TroughButton = kTroughPos.button;
        this.LowAlgaeButton = kLowAlgaePos.button;
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
            //ResetEncoderButton.onTrue(new EndEffectorEncoderResetCommand(endEffector));
            ResetEncoderButton.onTrue(new InstantCommand(() -> endEffector.resetEncoder()));

            //Coral
            ZeroPositionButton.onTrue(new EndEffectorButtonCommand(EndEffectorPosition.kZeroPositionAngle, endEffector));
            TroughButton.onTrue(new EndEffectorButtonCommand(EndEffectorPosition.kTroughAngle, endEffector));
            L2Button.onTrue(new EndEffectorButtonCommand(EndEffectorPosition.kL2Angle, endEffector));
            L3Button.onTrue(new EndEffectorButtonCommand(EndEffectorPosition.kMiddleAngle, endEffector));
            StationButton.onTrue(new EndEffectorButtonCommand(EndEffectorPosition.kStationAngle, endEffector));
            L4Button.onTrue(new EndEffectorButtonCommand(EndEffectorPosition.kL4Angle, endEffector));   
            
            //Algae
            NetButton.onTrue(new EndEffectorButtonCommand(EndEffectorPosition.kNetAngle, endEffector));
            HighAlgaeButton.onTrue(new EndEffectorButtonCommand(EndEffectorPosition.kHighAlgae, endEffector));
            LowAlgaeButton.onTrue(new EndEffectorButtonCommand(EndEffectorPosition.kLowAlgae, endEffector)); 
            
            //Rollers
            RollerIntake.onTrue(new InstantCommand(() -> endEffector.runRoller(kRollerIntakeSpeed)));
            RollerExtake.onTrue(new InstantCommand(() -> endEffector.runRoller(kRollerEjectSpeed)));
            RollerIntake.onFalse(new InstantCommand(() -> endEffector.stopRoller()));
            RollerExtake.onFalse(new InstantCommand(() -> endEffector.stopRoller()));

            //RollerIntake.onTrue(new EndEffectorRollerIntakeCommand(endEffector));
            //RollerExtake.onTrue(new EndEffectorRollerOutputCommand(endEffector));
            //RollerIntake.onFalse(new EndEffectorRollerStopCommand(endEffector));
            //RollerExtake.onFalse(new EndEffectorRollerStopCommand(endEffector));   
        }
    }
}
    
