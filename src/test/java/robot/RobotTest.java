package robot;

import static org.junit.Assert.assertEquals;

import org.junit.Test;
import frc.robot.Robot;

public class RobotTest {
    @Test
    public void testDeadband() {
        final double delta = 0.00001;
        assertEquals(-0, Robot.deadband(0, 0.2), delta);
        assertEquals(-1, Robot.deadband(-1, 0.2), delta);
        assertEquals(1, Robot.deadband(1, 0.2), delta);
        assertEquals(0, Robot.deadband(0.2, 0.2), delta);
        assertEquals(0, Robot.deadband(-0.2, 0.2), delta);
        assertEquals(0, Robot.deadband(0.1, 0.2), delta);
        assertEquals(0, Robot.deadband(-0.1, 0.2), delta);
        assertEquals(0.5, Robot.deadband(0.6, 0.2), delta);
        assertEquals(-0.5, Robot.deadband(-0.6, 0.2), delta);
    }
}
