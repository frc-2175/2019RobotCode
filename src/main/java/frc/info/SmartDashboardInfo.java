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
		putNumber(CARGO_BOTTOM_SETPOINT, 1, 1);
		putNumber(CARGO_MIDDLE_SETPOINT, 2, 2);
		putNumber(CARGO_TOP_SETPOINT, 3, 3);
		putNumber(HATCH_BOTTOM_SETPOINT, 1, 1);
		putNumber(HATCH_MIDDLE_SETPOINT, 2, 2);
		putNumber(HATCH_TOP_SETPOINT, 3, 3);
		putNumber(CARGO_INTAKE_ROLL_IN_ROLLERBAR_SPEED, 1, 1);
		putNumber(CARGO_INTAKE_ROLL_IN_BOX_MOTOR_SPEED, .75, .75);
		putNumber(CARGO_INTAKE_ROLL_OUT_ROLLERBAR_SPEED, -1, -1);
		putNumber(CARGO_INTAKE_ROLL_OUT_BOX_MOTOR_SPEED, -1, -1);
		putNumber(HATCH_INTAKE_SPIN_IN_FRONT, .3, .3);
		putNumber(HATCH_INTAKE_SPIN_OUT_FRONT, -.3, -.3);
		putNumber(HATCH_INTAKE_SPIN_IN_BACK, .3, .3);
		putNumber(HATCH_INTAKE_SPIN_OUT_BACK, -.3, -.3);
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
