package frc.command.autonomous;

import edu.wpi.first.wpilibj.Timer;
import frc.ServiceLocator;
import frc.command.Command;
import frc.subsystem.CargoIntakeSubsystem;

public class SpitOutCargoCommand extends Command {
    private CargoIntakeSubsystem cargoIntakeSubsystem;
    private double timeRollingOut;
    private double startTime;
    private double nowTime;


    /**
     * 
     * @param timeRollingOut uhh time is in 
     */
    public SpitOutCargoCommand(double timeRollingOut) {
        this.timeRollingOut = timeRollingOut;
        cargoIntakeSubsystem = ServiceLocator.get(CargoIntakeSubsystem.class);
    }

    public void init() {
        startTime = Timer.getFPGATimestamp();
    }

    public void execute() {
        nowTime = Timer.getFPGATimestamp();
        cargoIntakeSubsystem.solenoidIn();
        cargoIntakeSubsystem.rollOut();
    }

    public boolean isFinished() {
        return timeRollingOut <= (nowTime - startTime); //if time has run for as long or more long than we want, stop !!! : )
    }

    public void end() {
        cargoIntakeSubsystem.solenoidOut();
        cargoIntakeSubsystem.stopAllMotors();
    }
}
