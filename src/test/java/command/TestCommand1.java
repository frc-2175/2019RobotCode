package command;

import frc.command.Command;

public class TestCommand1 extends Command {
	private int timesExecuted = 0;
	private boolean hasInitialized = false;
	private boolean hasEnded = false;
	public void init() {
		hasInitialized = true;
	}

	public void execute() {
		timesExecuted++;
	}

	public boolean isFinished() {
		return timesExecuted >= 3;
	}

	public void end() {
		hasEnded = true;
	}

	public int getTimes() {
		return timesExecuted;
	}

	public boolean getHasInitalized() {
		return hasInitialized;
	}

	public boolean getHasEnded() {
		return hasEnded;
	}
}
