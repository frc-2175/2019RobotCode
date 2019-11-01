package frc.command;

import frc.spacetime.SpacetimeEvent;

/**
 * Runs commands sequentially (one after another). This command will end when
 * the last command ends.
 */
public class SequentialCommand extends Command {
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
			commands[index]._init();
		}
    }

    public void execute() {
        if(!isEmpty) {
			// Execute
			commands[index]._execute();

			// Check isFinished + transition
			if(commands[index]._isFinished()) {
                System.out.println("Ending a command (sequential)");
                commands[index]._end();
                if(index < commands.length - 1) {
                    System.out.println("Starting a new command");
					index += 1;
					commands[index]._init();
                }
            }
        }
    }

    public boolean isFinished() {
        boolean isFinished = index == commands.length - 1 && commands[commands.length - 1]._isFinished();
        return isEmpty ? true : isFinished;
    }

    public void end() {
        commands[commands.length - 1]._end();
	}

	@Override
	public void initSpacetimeEvent(SpacetimeEvent parentEvent) {
		super.initSpacetimeEvent(parentEvent);
		for(Command command : commands) {
			command.initSpacetimeEvent(event);
		}
	}
}
