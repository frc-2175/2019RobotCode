package frc.command.autonomous;

import edu.wpi.first.wpilibj.Timer;
import frc.ServiceLocator;
import frc.command.Command;
import frc.subsystem.HatchIntakeSubsystem;

public class HatchOuttakeCommand implements Command {
    private HatchIntakeSubsystem hatchIntakeSubsystem;
    private double timeSpinningOut; //it's in seconds by the way !!!!!!!!!!
    private double startTime;
    private double nowTime;

    public HatchOuttakeCommand(double timeSpinningOut) {
        this.timeSpinningOut = timeSpinningOut;
        hatchIntakeSubsystem = ServiceLocator.get(HatchIntakeSubsystem.class);
    }

    public void init() {
        startTime = Timer.getFPGATimestamp();
    }

    public void execute() {
        nowTime = Timer.getFPGATimestamp();
        hatchIntakeSubsystem.spinOutFront();
    }

    public boolean isFinished() {
        return (nowTime - startTime) >= timeSpinningOut;

    }

    public void end() {
        hatchIntakeSubsystem.stopSpinning();
    }

}