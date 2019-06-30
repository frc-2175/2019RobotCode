package frc.command;

import frc.ServiceLocator;
import frc.subsystem.DrivetrainSubsystem;

public interface Command {

    /**
     * Runs on the start of the command.
     */
    public void init();

    /**
     * Runs periodically after the init call.
     */
    public void execute();
    
    /**
     * @return whether or not the command is finished
     */
    public boolean isFinished();

    /**
     * Runs after the command ends.
     */
    public void end();
}