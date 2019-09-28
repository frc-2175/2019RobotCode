package frc.command;

public class CommandRunner {
    private Command command;
    private boolean hasRunInit;
    private boolean hasRunEnd;

    public CommandRunner(Command command) {
        this.command = command;
        hasRunInit = false;
        hasRunEnd = false;
    }

    public void runCommand() {
        if(!hasRunInit) {
            command.init();
            hasRunInit = true;
            command.execute();
        } else {
            if(!command.isFinished()) {
                command.execute();
            } else {
                if(!hasRunEnd) {
                    command.end();
                    hasRunEnd = true;
                }
            }
        }
    }

    public void resetCommand() {
        hasRunInit = false;
        hasRunEnd = false;
    }
}