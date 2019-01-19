package frc.subsystem;

import frc.MotorWrapper;
import frc.SolenoidWrapper;
import frc.ServiceLocator;
import frc.info.RobotInfo;

public class HatchIntakeSubsystem {
	private MotorWrapper rollerBarMotor;
	private MotorWrapper groundRollerBallMotor;
	private MotorWrapper groundActuationMotor;
	 private SolenoidWrapper actuatorHatchSolenoid;

	public HatchIntakeSubsystem() {
		ServiceLocator.register(this);

		RobotInfo robotInfo = ServiceLocator.get(RobotInfo.class);

		rollerBarMotor = robotInfo.get(RobotInfo.HATCH_ROLLER_BAR_MOTOR);
		groundRollerBallMotor = robotInfo.get(RobotInfo.GROUND_ROLLER_BAR_MOTOR);
		groundActuationMotor = robotInfo.get(RobotInfo.GROUND_ACTUATOR_MOTOR);
		actuatorHatchSolenoid = robotInfo.get(RobotInfo.HATCH_ACTUATOR_SOLENOID);
	}

	public void spinIn() {
		rollerBarMotor.set(0.3);
	}
	public void spinInGround(){
		groundRollerBallMotor.set(.3);
	}
	public void spinOutGround(){
		groundRollerBallMotor.set(-.3);
	}
	public void spinOut() {
		rollerBarMotor.set(-0.3);
	}
	
	public void stopSpinning() {
		groundRollerBallMotor.set(0);
		rollerBarMotor.set(0);
	}
	/*public void setGroundIntakeUp(){
		groundActuationMotor.set();
	 }*/
	 public void setHatchIntakeUp() {
	 	actuatorHatchSolenoid.set(true);
	 }

	public void setHatchIntakeDown() {
	 	actuatorHatchSolenoid.set(false);
	 }

}