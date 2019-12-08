package.frc.command.autonomous;

public class ElevatorUpCommand extends Command {
   public void init() {
       ElevatorSubsystem.setIsManual(false);
   } 
   public void execute() {
       ElevatorSubsystem.CargoPlaceElevatorShip();
   }
   public boolean isFinished() {

   }
   public void end() {

   }
}
