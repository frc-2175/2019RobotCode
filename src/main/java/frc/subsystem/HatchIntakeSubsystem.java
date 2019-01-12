package frc.subsystem;

import frc.MotorWrapper;
import frc.SolenoidWrapper;
import frc.ServiceLocator;
import frc.info.*;

public class HatchIntakeSubsystem {
	private MotorWrapper rollerBarMotor;
	private SolenoidWrapper actuatorSolenoid;

	public HatchIntakeSubsystem() {
		ServiceLocator.register(this);

		RobotInfo robotInfo = ServiceLocator.get(RobotInfo.class);

		rollerBarMotor = robotInfo.get(RobotInfo.HATCH_ROLLER_BAR_MOTOR);
		actuatorSolenoid = robotInfo.get(RobotInfo.HATCH_ACTUATOR_SOLENOID);
	}

	public void spinIn() {
		rollerBarMotor.set(0.3);
	}

	public void spinOut() {
		rollerBarMotor.set(-0.3);
	}

	public void setIntakeUp() {
		actuatorSolenoid.set(true);
	}

	public void setIntakeDown() {
		actuatorSolenoid.set(false);
	}

}