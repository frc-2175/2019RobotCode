package command;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertTrue;

import org.junit.Test;
import org.junit.Before;

import frc.ServiceLocator;
import frc.command.Command;
import frc.command.SequentialCommand;
import frc.logging.LogHandler;
import frc.logging.Logger;
import frc.logging.StdoutHandler;

public class SequentialCommandTest {
	@Before
	public void before() {
		Logger robotLogger = new Logger(new LogHandler[] {
			new StdoutHandler()
		});

		ServiceLocator.clear();
		ServiceLocator.register(robotLogger);
	}

	@Test
	public void testInitialization() {
		TestCommand testCommand1 = new TestCommand();
		TestCommand testCommand2 = new TestCommand();
		Command[] commands = { testCommand1, testCommand2 };
		SequentialCommand seqCommand = new SequentialCommand(commands);
		seqCommand.init();
		assertTrue("The first command did not initialize", testCommand1.getHasInitalized());
		assertTrue("The second command did initialize", !testCommand2.getHasInitalized());
	}

	@Test
	public void testExecution() {
		TestCommand testCommand1 = new TestCommand();
		TestCommand testCommand2 = new TestCommand();
		Command[] commands = { testCommand1, testCommand2 };
		SequentialCommand seqCommand = new SequentialCommand(commands);

		seqCommand.init();

		seqCommand.execute();
		assertEquals(1, testCommand1.getTimes());
		assertEquals(0, testCommand2.getTimes());

		seqCommand.execute();
		seqCommand.execute();
		assertEquals(3, testCommand1.getTimes());
		assertEquals(0, testCommand2.getTimes());

		testCommand1.setIsFinished(true);
		seqCommand.execute();
		assertEquals(4, testCommand1.getTimes());
		assertEquals(0, testCommand2.getTimes());
		assertTrue("First command did not end", testCommand1.getHasEnded());
		assertTrue("First command did not finish", testCommand1.isFinished());
		assertTrue("Second command did not initialize", testCommand2.getHasInitalized());

		seqCommand.execute();
		seqCommand.execute();
		assertEquals(4, testCommand1.getTimes());
		assertEquals(2, testCommand2.getTimes());

		testCommand2.setIsFinished(true);
		seqCommand.execute();
		assertTrue("Second command did not end", testCommand2.getHasEnded());
		assertTrue("Second command did not finish", testCommand2.isFinished());
	}
}
