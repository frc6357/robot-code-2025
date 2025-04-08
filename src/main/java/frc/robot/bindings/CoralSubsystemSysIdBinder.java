package frc.robot.bindings;

import static frc.robot.Ports.OperatorPorts.kOperatorA;
import static frc.robot.Ports.OperatorPorts.kOperatorB;
import static frc.robot.Ports.OperatorPorts.kOperatorX;
import static frc.robot.Ports.OperatorPorts.kOperatorY;

import java.util.Optional;

import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.subsystems.CoralSubsystem;

public class CoralSubsystemSysIdBinder implements CommandBinder {

    private final Optional<CoralSubsystem> coralSubsystem;

    public CoralSubsystemSysIdBinder(Optional<CoralSubsystem> coralSubsystem) {
        this.coralSubsystem = coralSubsystem;
    }

    @Override
    public void bindButtons() {
        coralSubsystem.ifPresent((coral) -> {
    
            /*
            * Joystick Y = quasistatic forward
            * Joystick A = quasistatic reverse
            * Joystick B = dynamic forward
            * Joystick X = dyanmic reverse
            */
            kOperatorY.button.whileTrue(coral.sysIdQuasistatic(SysIdRoutine.Direction.kForward));
            kOperatorA.button.whileTrue(coral.sysIdQuasistatic(SysIdRoutine.Direction.kReverse));
            kOperatorB.button.whileTrue(coral.sysIdDynamic(SysIdRoutine.Direction.kForward));
            kOperatorX.button.whileTrue(coral.sysIdDynamic(SysIdRoutine.Direction.kReverse));
        });
    }
    
}
