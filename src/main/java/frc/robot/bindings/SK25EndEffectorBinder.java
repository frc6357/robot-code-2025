package frc.robot.bindings;

// Joystick constants
import static frc.robot.Konstants.EndEffectorConstants.kJoystickDeadband;
import static frc.robot.Konstants.EndEffectorConstants.kJoystickReversed;
//import static frc.robot.Konstants.EndEffectorConstants.kRollerSpeed;
import static frc.robot.Ports.OperatorPorts.kEndEffectorAxis;
import static frc.robot.Ports.OperatorPorts.kFloorAlgae;
import static frc.robot.Ports.OperatorPorts.kHighAlgae;
// Operator ports
import static frc.robot.Ports.OperatorPorts.kIntake;
import static frc.robot.Ports.OperatorPorts.kIntakePos;
import static frc.robot.Ports.OperatorPorts.kLowAlgae;
import static frc.robot.Ports.OperatorPorts.kLowBranchEffector;
import static frc.robot.Ports.OperatorPorts.kMiddleBranchEffector;
import static frc.robot.Ports.OperatorPorts.kNetPos;
import static frc.robot.Ports.OperatorPorts.kShoot;
import static frc.robot.Ports.OperatorPorts.kTopBranchEffector;
import static frc.robot.Ports.OperatorPorts.kTroughEffector;
import static frc.robot.Ports.OperatorPorts.kZeroPositionOperator;
import static frc.robot.Ports.OperatorPorts.resetencoder;

// Misc.
import java.util.Optional;

// Relative encoder (REV)
import com.revrobotics.RelativeEncoder;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.WaitCommand;
//import edu.wpi.first.wpilibj2.command.WaitCommand;
//import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Konstants.EndEffectorConstants.EndEffectorPosition;

// Unused imports

//import static frc.robot.Konstants.ElevatorConstants.ElevatorPosition.kLowPosition;
//import static frc.robot.Konstants.ElevatorConstants.ElevatorPosition.kTopPosition;
//import static frc.robot.Konstants.ElevatorConstants.ElevatorPosition.kTroughPosition;
//import static frc.robot.Konstants.ElevatorConstants.ElevatorPosition.kZeroPosition;
//import static frc.robot.Konstants.EndEffectorConstants.EndEffectorPosition.*;
//import frc.robot.commands.EndEffectorEncoderResetCommand;
//import frc.robot.commands.EndEffectorStop;
//import static frc.robot.Konstants.EndEffectorConstants.kJoystickChange;
//import frc.robot.Konstants.EndEffectorConstants.EndEffectorPosition;
import frc.robot.commands.EndEffectorButtonCommand;
// Intake / eject commands
import frc.robot.commands.EndEffectorJoystickCommand;
import frc.robot.commands.EndEffectorRollerIntakeCommand;
import frc.robot.commands.EndEffectorRollerOutputCommand;
import frc.robot.commands.EndEffectorRollerStopCommand;
import frc.robot.subsystems.CoralSubsystem;
import frc.robot.subsystems.SK25EndEffector;
import frc.robot.utils.filters.DeadbandFilter;

public class SK25EndEffectorBinder implements CommandBinder {

    Optional<SK25EndEffector> endEffectorSubsystem;
    Optional<CoralSubsystem> elevator;

    Trigger LowButton;
    Trigger MiddleButton;
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
    Trigger Net;
    Trigger HighAlgae;
    Trigger LowAlgae;
    Trigger floorAlgae;

    RelativeEncoder mEncoder;

    public SK25EndEffectorBinder(Optional<SK25EndEffector> endEffectorSubsystem, Optional<CoralSubsystem> elevator)
    {
        this.endEffectorSubsystem   = endEffectorSubsystem;
        this.elevator = elevator;
        this.ResetEncoderButton     = resetencoder.button;
        this.RollerIntake           = kIntake.button;
        this.RollerOutPut           = kShoot.button;
        this.zeroPositionButton = kZeroPositionOperator.button;
        this.LowButton = kLowBranchEffector.button.or(kMiddleBranchEffector.button);
        this.MiddleButton = kMiddleBranchEffector.button;
        this.IntakeButton = kIntakePos.button;
        this.TopButton = kTopBranchEffector.button;
        this.TroughButton = kTroughEffector.button;
        this.LowAlgae = kLowAlgae.button;
        this.HighAlgae = kHighAlgae.button;
        this.Net = kNetPos.button;
        this.floorAlgae = kFloorAlgae.button;
    }

    public void bindButtons()
    {
        // If subsystem is present then this method will bind the buttons
        if (endEffectorSubsystem.isPresent())
        {
            SK25EndEffector endEffector = endEffectorSubsystem.get();
            CoralSubsystem m_elevator = elevator.get();

            double joystickGain = kJoystickReversed ? -1 : 1;
            kEndEffectorAxis.setFilter(new DeadbandFilter(kJoystickDeadband, joystickGain));
            
            // ResetEncoderButton.onTrue(new EndEffectorEncoderResetCommand(endEffector)); // TODO: Add back reset encoder command? This was originally for use with NEO Vortex pivot motor
            zeroPositionButton.onTrue(new EndEffectorButtonCommand(EndEffectorPosition.kZeroPositionAngle, endEffector));
            TroughButton.onTrue(new EndEffectorButtonCommand(EndEffectorPosition.kTroughPositionAngle, endEffector));
            LowButton.onTrue(new EndEffectorButtonCommand(EndEffectorPosition.kLowPositionAngle, endEffector));
            MiddleButton.onTrue(new EndEffectorButtonCommand(EndEffectorPosition.kMiddleAngle, endEffector));
            IntakeButton.onTrue(new EndEffectorButtonCommand(EndEffectorPosition.kIntakePositionAngle, endEffector));
            TopButton.onTrue(new EndEffectorButtonCommand(EndEffectorPosition.kTopPositionAngle, endEffector));   
            
            Net.onTrue(new EndEffectorButtonCommand(EndEffectorPosition.kNetAngle, endEffector));
            HighAlgae.onTrue(new EndEffectorButtonCommand(EndEffectorPosition.kHighAlgae, endEffector));
            LowAlgae.onTrue(new EndEffectorButtonCommand(EndEffectorPosition.kLowAlgae, endEffector));
            floorAlgae.onTrue(Commands.sequence(new WaitCommand(0.5), new EndEffectorButtonCommand(EndEffectorPosition.kFloorAngle, endEffector)));
            
            
            //RollerIntake.onTrue(new EndEffectorRollerIntakeCommand(endEffector));
            RollerIntake.onTrue(new EndEffectorRollerIntakeCommand(endEffector, m_elevator));
            RollerOutPut.onTrue(new EndEffectorRollerOutputCommand(endEffector, m_elevator));
            RollerIntake.onFalse(new EndEffectorRollerStopCommand(endEffector));
            RollerOutPut.onFalse(new EndEffectorRollerStopCommand(endEffector));

            /*
            RollerIntake.whileTrue(new WaitUntilCommand(endEffector::haveCoral)
                    .beforeStarting(() -> endEffector.runRoller(-kRollerSpeed), endEffector)
                    .andThen(new WaitCommand(.23))
                    .finallyDo(() -> endEffector.runRoller(0)));
            */

            endEffector.setDefaultCommand(
                    // Vertical movement of the arm is controlled by the Y axis of the right stick.
                    // Up on joystick moving arm up and down on stick moving arm down.
                  new EndEffectorJoystickCommand(
                        () -> {return kEndEffectorAxis.getFilteredAxis();},
                       endEffector));

           
        }
    }
}
    
