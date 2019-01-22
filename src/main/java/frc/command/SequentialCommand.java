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
        index = 0;
        if(!isEmpty) {
            commands[0].init();
        }
    }

    public void execute() {
        if(!isEmpty) {
            if(commands[index].isFinished()) {
                System.out.println("Ending a command");
                commands[index].end();
                if(index < commands.length - 1) {
                    System.out.println("Starting a new command");
                    index += 1;
                    commands[index].init();
                }
            } else {
                commands[index].execute();
            }
        }
    }

    public boolean isFinished() {
        boolean isFinished = index == commands.length - 1 && commands[commands.length - 1].isFinished();
        return isEmpty ? true : isFinished;
    }

    public void end() {
        commands[commands.length - 1].end();
    }
}
