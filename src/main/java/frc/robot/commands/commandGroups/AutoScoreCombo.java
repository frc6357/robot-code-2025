package frc.robot.commands.commandGroups;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Konstants.ElevatorConstants.CoralSubsystemConstants.ElevatorSetpoints;
import frc.robot.Konstants.EndEffectorConstants.EndEffectorPosition;
import frc.robot.commands.EndEffectorRollerIntakeCommand;
import frc.robot.commands.EndEffectorRollerStopCommand;
import frc.robot.commands.AlignToReefTag;
import frc.robot.subsystems.CoralSubsystem;
import frc.robot.subsystems.SK25EndEffector;
import frc.robot.subsystems.SKSwerve;
import frc.robot.subsystems.vision.SK25Vision;

import static frc.robot.Konstants.EndEffectorConstants.kRollerSuperSpeed;
import static frc.robot.commands.AlignToReefTag.Target;
import static frc.robot.subsystems.vision.SK25Vision.DriveToPose;
import static frc.robot.subsystems.vision.SK25Vision.RotateToPose;

public class AutoScoreCombo extends SequentialCommandGroup {

    public static enum ScoreType {
        CORAL,
        ALGAE
    }

    public AutoScoreCombo(
        Target target,
        ScoreType scoreType,
        SKSwerve m_swerve, 
        SK25Vision m_vision, 
        CoralSubsystem m_elevator, 
        SK25EndEffector m_endEffector) 
    {
        addRequirements(m_swerve, m_elevator, m_endEffector);
        switch(scoreType) {
            // If manipulating Coral
            case CORAL:
                addCommands(
                    // Align to Tag with a 1.5 second timeout
                    Commands.race(
                        new AlignToReefTag(
                            target, 
                            DriveToPose.getConfig(), 
                            RotateToPose.getConfig(), 
                            m_vision, 
                            m_swerve),
                        Commands.waitSeconds(1.5)
                    ),
                    // Wait for elevator to reach target
                    Commands.waitUntil(()-> m_elevator.atSetpoint()),
                    // Use end effector to score (ripped straight from SK25EndEffectorBinder)
                    Commands.either(
                        Commands.parallel(
                            new EndEffectorRollerIntakeCommand(m_endEffector, m_elevator),
                            Commands.sequence(
                                Commands.waitSeconds(0.25),
                                new InstantCommand(() -> m_endEffector.setTargetAngle(EndEffectorPosition.kIntakePositionAngle))
                            )
                        ),
                        new EndEffectorRollerIntakeCommand(m_endEffector, m_elevator),
                        () -> {
                            return (m_elevator.elevatorCurrentTarget == ElevatorSetpoints.kLevel4 && m_endEffector.isL4());
                        }
                    ),
                    // Wait for a split second before turning off rollers
                    Commands.waitSeconds(0.15),
                    // Stop the rollers
                    new EndEffectorRollerStopCommand(m_endEffector)
                );
                break;

            // If manipulating Algae
            case ALGAE: 
                addCommands(
                    // Constantly run the rollers to intake the algae while we drive up to the reef
                    Commands.deadline(
                        Commands.sequence(
                            // Wait until elevator is ready for algae pickup to align
                            Commands.waitUntil(() -> m_elevator.atSetpoint())),
                            // Align to Tag with a 1.5 second timeout
                            Commands.race(
                                new AlignToReefTag(
                                    target, 
                                    DriveToPose.getConfig(), 
                                    RotateToPose.getConfig(), 
                                    m_vision, 
                                    m_swerve),
                                Commands.waitSeconds(1.5)),
                        new InstantCommand(()-> m_endEffector.runRoller(kRollerSuperSpeed))
                    ),
                    // Wait for a split second to ensure algae is in place
                    Commands.waitSeconds(0.15),
                    // Stop the rollers
                    new EndEffectorRollerStopCommand(m_endEffector)
                );
        }
        
    }
}
