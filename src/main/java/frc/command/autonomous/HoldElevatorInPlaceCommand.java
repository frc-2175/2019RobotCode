
package frc.command.autonomous;

import frc.ServiceLocator;
import frc.command.Command;
import frc.subsystem.ElevatorSubsystem;

public class HoldElevatorInPlaceCommand extends Command {

    ElevatorSubsystem elevatorSubsystem;

    public HoldElevatorInPlaceCommand() {
        elevatorSubsystem = ServiceLocator.get(ElevatorSubsystem.class);

    }

    public void init() {
        elevatorSubsystem.setIsManual(false);
    }

    public void execute() {

    }

    public boolean isFinished() {
        return true;
    }

    public void end() {
        elevatorSubsystem.setIsManual(true);
        elevatorSubsystem.manualMove(0.0);
    }
}
