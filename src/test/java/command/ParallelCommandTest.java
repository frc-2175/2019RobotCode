package command;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertTrue;

import org.junit.Test;
import frc.command.Command;
import frc.command.ParallelCommand;

public class ParallelCommandTest {
    @Test
	public void testInitialization() {
		TestCommand1 testCommand1 = new TestCommand1();
        TestCommand2 testCommand2 = new TestCommand2();
        Command[] commands = { testCommand1, testCommand2 };
        ParallelCommand parcommand = new ParallelCommand(commands);
        parcommand.init();
        assertTrue("The commads didn't initialize", testCommand1.getHasInitalized() && testCommand2.getHasInitalized());
    }        
    @Test
    public void testExecution() {
		TestCommand1 testCommand1 = new TestCommand1();
        TestCommand2 testCommand2 = new TestCommand2();
        Command[] commands = { testCommand1, testCommand2 }; 
        ParallelCommand parcommand = new ParallelCommand(commands);
        parcommand.init();
        parcommand.execute();
        assertEquals(2, testCommand1.getTimes() + testCommand2.getTimes());
        parcommand.execute();
        assertEquals(4, testCommand1.getTimes() + testCommand2.getTimes());
        parcommand.execute();
        assertEquals(5, testCommand1.getTimes() + testCommand2.getTimes());
        assertTrue("the second one did not end", testCommand2.getHasEnded());
        parcommand.execute();
        assertEquals(5, testCommand1.getTimes() + testCommand2.getTimes());
        
        assertTrue("The command didn't end", testCommand1.getHasEnded() && testCommand2.getHasEnded());
        assertTrue("The command didn't finish", testCommand1.isFinished() && testCommand2.isFinished());
    }

}
