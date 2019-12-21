package command;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertTrue;

import org.junit.Test;
import org.junit.Before;

import frc.ServiceLocator;
import frc.command.Command;
import frc.command.RunWhileCommand;;
import frc.logging.LogHandler;
import frc.logging.Logger;
import frc.logging.StdoutHandler;

public class RunWhileCommandTest {
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
		TestCommand primary = new TestCommand();
		TestCommand secondary = new TestCommand();

		RunWhileCommand command = new RunWhileCommand(primary, secondary);
		command.init();

		assertTrue("primary command didn't initialize", primary.getHasInitalized());
		assertTrue("secondary command didn't initialize", secondary.getHasInitalized());
	}

	@Test
	public void testPrimaryEndsFirst() {
		TestCommand primary = new TestCommand();
		TestCommand secondary = new TestCommand();

		RunWhileCommand command = new RunWhileCommand(primary, secondary);
		command.init();

		command.execute();
		assertEquals(1, primary.getTimes());
		assertEquals(1, secondary.getTimes());

		primary.setIsFinished(true);
		assertTrue("command is not finished", command._isFinished());

		command.end();
		assertTrue("primary command didn't end", primary.getHasEnded());
		assertTrue("secondary command didn't end", secondary.getHasEnded());
	}

	@Test
	public void testSecondaryEndsFirst() {
		TestCommand primary = new TestCommand();
		TestCommand secondary = new TestCommand();

		RunWhileCommand command = new RunWhileCommand(primary, secondary);
		command.init();

		command.execute();
		assertEquals(1, primary.getTimes());
		assertEquals(1, secondary.getTimes());

		secondary.setIsFinished(true);
		command.execute();
		assertEquals(2, primary.getTimes());
		assertEquals(2, secondary.getTimes());
		assertTrue("secondary command didn't end", secondary.getHasEnded());
		assertFalse("command thought it was finished when secondary ended", command._isFinished());

		command.execute();
		command.execute();
		assertEquals(4, primary.getTimes());
		assertEquals(2, secondary.getTimes());

		primary.setIsFinished(true);
		assertTrue("command is not finished", command._isFinished());

		primary.end();
		assertTrue("primary command didn't end", primary.getHasEnded());
		assertEquals("secondary command ended more than once", 1, secondary.getTimesEnded());
	}
}
