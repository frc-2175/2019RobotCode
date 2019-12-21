
package frc.command.autonomous;

import frc.ServiceLocator;
import frc.command.Command;
import frc.subsystem.ElevatorSubsystem;

public class ElevatorStayCommand extends Command {

    ElevatorSubsystem elevatorSubsystem;

    public ElevatorStayCommand() {
        elevatorSubsystem = ServiceLocator.get(ElevatorSubsystem.class);

    }

    public void init() {
        elevatorSubsystem.setIsManual(true);
        elevatorSubsystem.setSetpoint(elevatorSubsystem.getElevatorPosition());
    }

    public void execute() {
        elevatorSubsystem.setElevator();
    }

    public boolean isFinished() {
        return false;
    }

    public void end() {
        elevatorSubsystem.manualMove(0.0);
    }
}
