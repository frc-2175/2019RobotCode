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
        pidControllerP.clear();
        assertEquals(10, pidControllerP.pid(0, 10), 0.0001);
        pidControllerP.updateTime(0.001);
        assertEquals(7, pidControllerP.pid(3, 10), 0.0001);
        pidControllerP.updateTime(0.001);
        assertEquals(5, pidControllerP.pid(5, 10), 0.0001);
        pidControllerP.updateTime(0.001);


	}

	@Test
	public void testIntegral() {
        PIDController pidControllerI = new PIDController(0, ki, 0);
        pidControllerI.clear();
        pidControllerI.updateTime(0.05);
        assertEquals(0.4, pidControllerI.pid(2, 10), 0.0001);
        pidControllerI.updateTime(.1);
        assertEquals(0.8, pidControllerI.pid(6, 10), 0.0001);
	}

	@Test
	public void testDerivative() {
		PIDController pidControllerD = new PIDController(0, 0, kd);
        pidControllerD.clear();
        assertEquals(0, pidControllerD.pid(0, 10), 0.0001);
        pidControllerD.updateTime(0.05);
        assertEquals(-40, pidControllerD.pid(2, 10), 0.0001);
        pidControllerD.updateTime(0.05);
        assertEquals(-60, pidControllerD.pid(5, 10), 0.0001);
        pidControllerD.updateTime(0.05);
        assertEquals(-80, pidControllerD.pid(9, 10), 0.0001);
	}

	@Test
	public void testZeroDT() {
		PIDController pidController = new PIDController(kp, ki, kd);
		pidController.clear();
		pidController.updateTime(0);
		pidController.pid(0, 10);
	}
}
