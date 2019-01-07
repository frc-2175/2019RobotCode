package frc.command;

/**
 * Runs commands sequentially (one after another).
 * This command will end when the last command ends.
 */
public class SequentialCommand implements Command {
    private final Command[] commands;
    private int index = 0;

    /**
     * @param commands an array of the commands to be run sequentially
     */
    public SequentialCommand(Command[] commands) {
        this.commands = commands;
    }

    public void init() {
        for(Command command : commands) {
            command.init();
        }
    }

    public void execute() {
        if(commands[index].isFinished()) {
            commands[index].end();
            if(index < commands.length - 1) {
                index += 1;
            }
        } else {
            commands[index].execute();
        }
    }

    public boolean isFinished() {
        return commands[commands.length - 1].isFinished();
    }

    public void end() {
        // Doesn't really do much in this situation
    }
}
