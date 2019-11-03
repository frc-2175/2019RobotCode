package command;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertTrue;

import org.junit.Test;

import frc.command.Command;
import frc.command.SequentialCommand;

public class SequentialCommandTest {
	@Test
	public void testInitialization() {
		TestCommand1 testCommand1 = new TestCommand1();
		TestCommand2 testCommand2 = new TestCommand2();
		Command[] commands = { testCommand1, testCommand2 };
		SequentialCommand seqCommand = new SequentialCommand(commands);
		seqCommand.init();
		assertTrue("The first command did not initialize", testCommand1.getHasInitalized());
		assertTrue("The second command did initialize", !testCommand2.getHasInitalized());
	}

	@Test
	public void testExecution() {
		TestCommand1 testCommand1 = new TestCommand1();
		TestCommand2 testCommand2 = new TestCommand2();
		Command[] commands = { testCommand1, testCommand2 };
		SequentialCommand seqCommand = new SequentialCommand(commands);
		seqCommand.init();
		seqCommand.execute();
		assertEquals(1, testCommand1.getTimes());
		seqCommand.execute();
		seqCommand.execute();
		seqCommand.execute();
		assertTrue("First command did not end", testCommand1.getHasEnded());
		assertTrue("First command did not finish", testCommand1.isFinished());
		seqCommand.execute();
		assertEquals(2, testCommand2.getTimes());
		seqCommand.execute();
		assertTrue("Second command did not end", testCommand2.getHasEnded());
		assertTrue("Second command did not finish", testCommand2.isFinished());
	}
}
