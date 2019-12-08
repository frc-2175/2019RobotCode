
package frc.command.autonomous;

import frc.ServiceLocator;
import frc.command.Command;
import frc.subsystem.ElevatorSubsystem;


public class ElevatorUpCommand extends Command {

    ElevatorSubsystem elevatorSubsystem;

   public ElevatorUpCommand() {
       elevatorSubsystem = ServiceLocator.get(ElevatorSubsystem.class);

    }

   public void init() {
       elevatorSubsystem.setIsManual(false);
   } 
   public void execute() {
       elevatorSubsystem.CargoPlaceElevatorShip();
   }
   public boolean isFinished() {
        if (elevatorSubsystem.getElevatorPosition() >= 31) {
            return true;
        } else {
            return false;
        }
   }
   public void end() {
        elevatorSubsystem.setIsManual(true);
        elevatorSubsystem.manualMove(0.0);
   }
}
