package frc.command;

public interface Command {
    /**
     * Runs on the start of the command.
     */
    public abstract void init();

    /**
     * Runs periodically after the init call.
     */
    public abstract void execute();
    
    /**
     * @return whether or not the command is finished
     */
    public abstract boolean isFinished();

    /**
     * Runs after the command ends.
     */
    public abstract void end();
}