package frc.robot.bindings;

import static frc.robot.Ports.DriverPorts.kAutoRotate;
import static frc.robot.Ports.DriverPorts.kEject;
import static frc.robot.Ports.DriverPorts.kGroundAlgae;
import static frc.robot.Ports.DriverPorts.kIntake;
import static frc.robot.Ports.DriverPorts.kIntakePos;
import static frc.robot.Ports.DriverPorts.kZeroPosition;

import java.util.Optional;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Konstants.EndEffectorConstants.EndEffectorPosition;
import frc.robot.commands.EndEffectorButtonCommand;
import frc.robot.commands.EndEffectorRollerIntakeCommand;
import frc.robot.commands.EndEffectorRollerOutputCommand;
import frc.robot.commands.EndEffectorRollerStopCommand;
import frc.robot.commands.RotateToTag;
import frc.robot.subsystems.CoralSubsystem;
import frc.robot.subsystems.SK25Climb;
import frc.robot.subsystems.SK25EndEffector;
import frc.robot.subsystems.SKSwerve;
import frc.robot.subsystems.CoralSubsystem.Setpoint;
import frc.robot.subsystems.vision.SK25Vision;

public class SK25OutreachBinder implements CommandBinder {

    private SKSwerve m_swerve;
    private SK25Climb m_climb;
    private CoralSubsystem m_coral;
    private SK25EndEffector m_endEffector;
    private SK25Vision m_vision;

    Trigger eject;
    Trigger intake;
    Trigger floorAlgae;
    Trigger autoRotate;
    Trigger intakePosition;
    Trigger zeroPosition;

    public SK25OutreachBinder(
        Optional<SKSwerve> m_swerveContainer, 
        Optional<SK25Vision> m_visionContainer, 
        Optional<CoralSubsystem> m_elevatorContainer,
        Optional<SK25EndEffector> m_endEffectorContainer,
        Optional<SK25Climb> m_climbContainer)
        {
            this.m_swerve = m_swerveContainer.get();
            this.m_vision = m_visionContainer.get();
            this.m_coral = m_elevatorContainer.get();
            this.m_endEffector = m_endEffectorContainer.get();
            this.m_climb = m_climbContainer.get();

            this.eject = kEject.button;
            this.intake = kIntake.button;
            this.floorAlgae = kGroundAlgae.button;
            this.intakePosition = kIntakePos.button;
            this.autoRotate = kAutoRotate.button;
            this.zeroPosition = kZeroPosition.button; 
        }


    // For outreach, we basically just mimic what RobotContainer does with binding buttons, 
    // but within a binder so that it all gets mapped with one controller
    @Override
    public void bindButtons() {
        SKSwerveBinder swerveBinder = new SKSwerveBinder(Optional.of(m_swerve));
        swerveBinder.bindButtons();

        autoRotate.toggleOnTrue(
            new RotateToTag(
                SK25Vision.RotateToPose.getConfig(), 
                m_vision, 
                m_swerve)
        );

        eject.onTrue(new EndEffectorRollerIntakeCommand(m_endEffector, m_coral));
        eject.onFalse(new EndEffectorRollerStopCommand(m_endEffector));

        intake.onTrue(new EndEffectorRollerOutputCommand(m_endEffector, m_coral));
        intake.onFalse(new EndEffectorRollerStopCommand(m_endEffector));

        floorAlgae.onTrue(
            Commands.parallel(
                Commands.sequence(
                    new WaitCommand(0.5), 
                    new EndEffectorButtonCommand(EndEffectorPosition.kFloorAngle, m_endEffector)
                ),
                m_coral.setSetpointCommand(Setpoint.kFloor)
            )
        );
        
        intakePosition.onTrue(
            Commands.parallel(
                new EndEffectorButtonCommand(EndEffectorPosition.kIntakePositionAngle, m_endEffector),
                m_coral.setSetpointCommand(Setpoint.kIntake)
            )
        );

        zeroPosition.onTrue(
            Commands.parallel(
                new EndEffectorButtonCommand(EndEffectorPosition.kZeroPositionAngle, m_endEffector),
                m_coral.setSetpointCommand(Setpoint.kZero)
            )
        );
    }

}
