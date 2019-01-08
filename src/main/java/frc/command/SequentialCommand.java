package frc.command;

/**
 * Runs commands sequentially (one after another).
 * This command will end when the last command ends.
 */
public class SequentialCommand implements Command {
    private final Command[] commands;
    private int index = 0;
    private boolean isEmpty;

    /**
     * @param commands an array of the commands to be run sequentially
     */
    public SequentialCommand(Command[] commands) {
        this.commands = commands;
        isEmpty = commands.length == 0;
    }

    public void init() {
        if(!isEmpty) {
            commands[0].init();
        }
    }

    public void execute() {
        if(!isEmpty) {
            if(commands[index].isFinished()) {
                commands[index].end();
                if(index < commands.length - 1) {
                    index += 1;
                    commands[index].init();
                }
            } else {
                commands[index].execute();
            }
        }
    }

    public boolean isFinished() {
        return isEmpty ? true : commands[commands.length - 1].isFinished();
    }

    public void end() {
        // Doesn't really do much in this situation
    }
}
