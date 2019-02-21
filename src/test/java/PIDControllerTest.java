import static org.junit.Assert.assertEquals;

import org.junit.Test;

import frc.PIDController;

public class PIDControllerTest {
	private final double kp = 1;
	private final double ki = 1;
	private final double kd = 1;

    @Test
    public void testProportional() {
        PIDController pidControllerP = new PIDController(kp, 0, 0);
        pidControllerP.clear(0);
		assertEquals(10, pidControllerP.pid(0, 10), 0.0001);
		pidControllerP.updateTime(1);
		assertEquals(7, pidControllerP.pid(3, 10), 0.0001);
        pidControllerP.updateTime(1);
        assertEquals(5, pidControllerP.pid(5, 10), 0.0001);
	}

	@Test
	public void testIntegral() {
        PIDController pidControllerI = new PIDController(0, ki, 0);
		pidControllerI.clear(0);
        pidControllerI.updateTime(0.05);
		assertEquals(0, pidControllerI.pid(2, 10), 0.0001);
        pidControllerI.updateTime(0.15);
		assertEquals(0.4, pidControllerI.pid(6, 10), 0.0001);
		pidControllerI.updateTime(0.2);
		assertEquals(0.45, pidControllerI.pid(9, 10), 0.0001);
	}

	@Test
	public void testDerivative() {
		PIDController pidControllerD = new PIDController(0, 0, kd);
        pidControllerD.clear(0);
        assertEquals(0, pidControllerD.pid(0, 10), 0.0001);
        pidControllerD.updateTime(0.05);
        assertEquals(-40, pidControllerD.pid(2, 10), 0.0001);
        pidControllerD.updateTime(0.1);
        assertEquals(-60, pidControllerD.pid(5, 10), 0.0001);
        pidControllerD.updateTime(0.15);
        assertEquals(-80, pidControllerD.pid(9, 10), 0.0001);
	}

	@Test
	public void testZeroDT() {
		PIDController pidController = new PIDController(kp, ki, kd);
		pidController.clear(0);
		pidController.updateTime(0);
		pidController.pid(0, 10);
	}
}
