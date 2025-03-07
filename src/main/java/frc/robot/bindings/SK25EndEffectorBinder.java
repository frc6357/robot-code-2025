package frc.robot.bindings;

// Intake / eject commands
import frc.robot.commands.EndEffectorJoystickCommand;
import frc.robot.commands.EndEffectorRollerIntakeCommand;
import frc.robot.commands.EndEffectorRollerOutputCommand;
import frc.robot.commands.EndEffectorRollerStopCommand;

// Joystick constants
import static frc.robot.Konstants.EndEffectorConstants.*;

// Operator ports
import static frc.robot.Ports.OperatorPorts.*;

// Relative encoder (REV)
import com.revrobotics.RelativeEncoder;

// Misc.
import java.util.Optional;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.subsystems.SK25EndEffector;
import frc.robot.utils.filters.DeadbandFilter;

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

public class SK25EndEffectorBinder implements CommandBinder {

    Optional<SK25EndEffector> endEffectorSubsystem;

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
    Trigger NetButton;

    RelativeEncoder mEncoder;

    public SK25EndEffectorBinder(Optional<SK25EndEffector> endEffectorSubsystem)
    {
        this.endEffectorSubsystem   = endEffectorSubsystem;
        this.ResetEncoderButton     = resetencoder.button;
        this.RollerIntake           = rollerintake.button;
        this.RollerOutPut           = rolleroutput.button;
        this.zeroPositionButton     = kZeroPositionOperatorEffector.button;
        this.LowMidButton           = kLowMidBranchEffector.button;
        this.IntakeButton           = kIntakePosEffector.button;
        this.TopButton              = kTopBranchEffector.button;
        this.TroughButton           = kTroughEffector.button;
        this.NetButton              = kNetEffector.button;
    }

    public void bindButtons()
    {
        // If subsystem is present then this method will bind the buttons
        if (endEffectorSubsystem.isPresent())
        {
            SK25EndEffector endEffector = endEffectorSubsystem.get();

            double joystickGain = kJoystickReversed ? -1 : 1;
            endArm.setFilter(new DeadbandFilter(kJoystickDeadband, joystickGain));
            
            // ResetEncoderButton.onTrue(new EndEffectorEncoderResetCommand(endEffector)); // TODO: Add back reset encoder command? This was originally for use with NEO Vortex pivot motor
            zeroPositionButton.onTrue(new EndEffectorButtonCommand(EndEffectorPosition.kZeroPositionAngle, endEffector));
            TroughButton.onTrue(new EndEffectorButtonCommand(EndEffectorPosition.kTroughPositionAngle, endEffector));
            LowMidButton.onTrue(new EndEffectorButtonCommand(EndEffectorPosition.kMidLowPositionAngle, endEffector));
            IntakeButton.onTrue(new EndEffectorButtonCommand(EndEffectorPosition.kIntakePositionAngle, endEffector));
            TopButton.onTrue(new EndEffectorButtonCommand(EndEffectorPosition.kTopPositionAngle, endEffector));  
            NetButton.onTrue(new EndEffectorButtonCommand(EndEffectorPosition.kNetPositionAngle, endEffector));      

            
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
    
