package frc.robot.bindings;

import static frc.robot.Ports.DriverPorts.kDSwapBack;
import static frc.robot.Ports.DriverPorts.kDSwapStart;
import static frc.robot.Ports.DriverPorts.killDriverRumble;
import static frc.robot.Ports.DriverPorts.swapDriverController;
import static frc.robot.Ports.OperatorPorts.kOSwapBack;
import static frc.robot.Ports.OperatorPorts.kOSwapStart;
import static frc.robot.Ports.OperatorPorts.killOperatorRumble;
import static frc.robot.Ports.OperatorPorts.swapOperatorController;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class SKHotswapBinder implements CommandBinder {

    Trigger operatorSwapStart;
    Trigger operatorSwapBack;
    Trigger driverSwapStart;
    Trigger driverSwapBack;

    public SKHotswapBinder() {
        this.operatorSwapBack = kOSwapBack.button; // Unused for now
        this.operatorSwapStart = kOSwapStart.button;
        this.driverSwapBack = kDSwapBack.button; // Unused for now
        this.driverSwapStart = kDSwapStart.button;
    }

    @Override
    public void bindButtons() {
        operatorSwapStart.debounce(3).onTrue(swapControllers());
        driverSwapStart.debounce(3).onTrue(swapControllers());

        operatorSwapStart.debounce(3).onFalse(finishSwapping());
        driverSwapStart.debounce(3).onFalse(finishSwapping());
    }

    private Command swapControllers() {
        return Commands.parallel(
            new InstantCommand(() -> swapOperatorController()), 
            new InstantCommand(() -> swapDriverController())
        );
    }

    private Command finishSwapping() {
        return Commands.parallel(
            new InstantCommand(() -> killDriverRumble()),
            new InstantCommand(() -> killOperatorRumble())
        );
    }

}
