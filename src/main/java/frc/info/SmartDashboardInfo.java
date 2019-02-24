package frc.info;

import java.util.Properties;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.ServiceLocator;

public class SmartDashboardInfo {
    private static final String PREFIX = "AutoPopulate/";
	private final boolean isComp;
	public static final String CARGO_BOTTOM_SETPOINT = "elevator.setpoint.cargo.bottom";
	public static final String CARGO_MIDDLE_SETPOINT = "elevator.setpoint.cargo.middle";
	public static final String CARGO_TOP_SETPOINT = "elevator.setpoint.cargo.top";
	public static final String HATCH_BOTTOM_SETPOINT = "elevator.setpoint.hatch.bottom";
	public static final String	HATCH_MIDDLE_SETPOINT = "elevator.setpoint.hatch.middle";
	public static final String HATCH_TOP_SETPOINT = "elevator.setpoint.hatch.top";
	public static final String CARGO_INTAKE_ROLL_IN_ROLLERBAR_SPEED = "intake.cargo.in.rollerbar";
	public static final String CARGO_INTAKE_ROLL_IN_BOX_MOTOR_SPEED = "intake.cargo.in.boxmotor";
	public static final String CARGO_INTAKE_ROLL_OUT_ROLLERBAR_SPEED = "intake.cargo.out.rollerbar";
	public static final String CARGO_INTAKE_ROLL_OUT_BOX_MOTOR_SPEED = "intake.cargo.out.boxmotor";
	public static final String HATCH_INTAKE_SPIN_IN_FRONT = "intake.hatch.front.in";
	public static final String HATCH_INTAKE_SPIN_OUT_FRONT = "intake.hatch.front.out";
	public static final String HATCH_INTAKE_SPIN_IN_BACK = "intake.hatch.back.in";
	public static final String HATCH_INTAKE_SPIN_OUT_BACK = "intake.hatch.back.out";
	public static final String ELEVATOR_PID_P = "elevator.pid.p";
	public static final String ELEVATOR_PID_I = "elevator.pid.i";
	public static final String ELEVATOR_PID_D = "elevator.pid.d";
	public static final String HATCH_PID_P = "hatch.pid.p";
	public static final String HATCH_PID_I = "hatch.pid.i";
	public static final String HATCH_PID_D = "hatch.pid.d";
	public static final String VISION_PID_P = "vision.pid.p";
	public static final String VISION_PID_I = "vision.pid.i";
	public static final String VISION_PID_D = "vision.pid.d";



    public SmartDashboardInfo() {
		ServiceLocator.register(this);
        Properties properties = RobotInfo.loadProperties("/home/lvuser/bot.properties");
        isComp = Boolean.parseBoolean((String) properties.get("isComp"));
        populate();
    }

    /**
     * This is where all smart dashboard values are put onto the dashboard
     * using the put methods. See below in {@link SmartDashboardInfo}
     */
    public void populate() {
		putNumber(CARGO_BOTTOM_SETPOINT, 16, 16);
		putNumber(CARGO_MIDDLE_SETPOINT, 44, 44);
		putNumber(CARGO_TOP_SETPOINT, 72, 72);
		putNumber(HATCH_BOTTOM_SETPOINT, 1, 1);
		putNumber(HATCH_MIDDLE_SETPOINT, 28, 28);
		putNumber(HATCH_TOP_SETPOINT, 56, 56);
		putNumber(CARGO_INTAKE_ROLL_IN_ROLLERBAR_SPEED, 0.65, 0.65);
		putNumber(CARGO_INTAKE_ROLL_IN_BOX_MOTOR_SPEED, .5, .5);
		putNumber(CARGO_INTAKE_ROLL_OUT_ROLLERBAR_SPEED, -1, -1);
		putNumber(CARGO_INTAKE_ROLL_OUT_BOX_MOTOR_SPEED, -0.7, -0.7);
		putNumber(HATCH_INTAKE_SPIN_IN_FRONT, 0.6, 0.6);
		putNumber(HATCH_INTAKE_SPIN_OUT_FRONT, -0.8, -0.8);
		putNumber(HATCH_INTAKE_SPIN_IN_BACK, 0.75, 0.75);
		putNumber(HATCH_INTAKE_SPIN_OUT_BACK, -0.4, -0.4);
		putNumber(ELEVATOR_PID_P, 1.0 / 36.0, 1.0 / 36.0);
		putNumber(ELEVATOR_PID_I, 1.0 / 30.0, 1.0 / 30.0);
		putNumber(ELEVATOR_PID_D, 0, 0);
		putNumber(HATCH_PID_P, 1.0 / 30.0, 1.0 / 30.0); //originally 1 / 30
		putNumber(HATCH_PID_I, 1.0/ 10.0, 1.0 / 10.0);
		putNumber(HATCH_PID_D, 0, 0);
		putNumber(VISION_PID_P, 1.0 / 36.0, 1.0 / 36.0);
		putNumber(VISION_PID_I, 0.01, 0.01);
		putNumber(VISION_PID_D, 0, 0);
    }

    public void putBoolean(String key, boolean comp, boolean practice) {
		SmartDashboard.putBoolean(PREFIX + key, isComp ? comp : practice);
	}

	public void putNumber(String key, double comp, double practice) {
		SmartDashboard.putNumber(PREFIX + key, isComp ? comp : practice);
	}

	public void putString(String key, String comp, String practice) {
		SmartDashboard.putString(PREFIX + key, isComp ? comp : practice);
	}

	public void putBooleanArray(String key, boolean[] comp, boolean[] practice) {
		SmartDashboard.putBooleanArray(PREFIX + key, isComp ? comp : practice);
	}

	public void putNumberArray(String key, double[] comp, double[] practice) {
		SmartDashboard.putNumberArray(PREFIX + key, isComp ? comp : practice);
	}

	public void putStringArray(String key, String[] comp, String[] practice) {
		SmartDashboard.putStringArray(PREFIX + key, isComp ? comp : practice);
	}

	public void putRaw(String key, byte[] comp, byte[] practice) {
		SmartDashboard.putRaw(PREFIX + key, isComp ? comp : practice);
	}

	public boolean getBoolean(String key) {
		return SmartDashboard.getBoolean(PREFIX + key, false);
	}

	public double getNumber(String key) {
		return SmartDashboard.getNumber(PREFIX + key, 0);
	}

	public String getString(String key) {
		return SmartDashboard.getString(PREFIX + key, "");
	}

	public boolean[] getBooleanArray(String key) {
		boolean[] defaultArray = { false };
		return SmartDashboard.getBooleanArray(PREFIX + key, defaultArray);
	}

	public double[] getNumberArray(String key) {
		double[] defaultArray = { 0 };
		return SmartDashboard.getNumberArray(PREFIX + key, defaultArray);
	}

	public String[] getStringArray(String key) {
		String[] defaultArray = { "" };
		return SmartDashboard.getStringArray(PREFIX + key, defaultArray);
	}

	public byte[] getRaw(String key) {
		byte[] defaultArray = { 0 };
		return SmartDashboard.getRaw(PREFIX + key, defaultArray);
	}
}
