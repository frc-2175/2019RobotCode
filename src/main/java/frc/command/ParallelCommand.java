package frc.command;

/**
 * Runs commands in parallel (at the same time).
 * This command will end when the last of the commands in parallel has ended.
 */
public class ParallelCommand implements Command {
    private final Command[] commands;
    private boolean[] hasEndRunYet;
    
    /**
     * @param commands an array of the commands to be run in parallel
     */
    public ParallelCommand(Command[] commands) {
        this.commands = commands;
        System.out.println("# Of Commands");
        System.out.println(commands.length);
        hasEndRunYet = new boolean[commands.length];
    }
    
    public void init() {
        for (Command command : commands) {
            command.init();
        }
    }

    public void execute() {
        for(int i = 0; i < commands.length; i++) {
            if(!commands[i].isFinished()) {
                commands[i].execute();
            } else if(!hasEndRunYet[i]) {
                System.out.println("Ending a command");
                commands[i].end();
                hasEndRunYet[i] = true;
            }
        }
    }

    public boolean isFinished() {
        boolean isFinished = true;
        for (Command command : commands) {
            if(!command.isFinished()) {
                isFinished = false;
            }
        }
        return isFinished;
    }

    public void end() {
        for(int i = 0; i < commands.length; i++) {
            if(!hasEndRunYet[i]) {
                System.out.println("Ending a command");
                commands[i].end();
                hasEndRunYet[i] = true;
            }
        }
    }
}