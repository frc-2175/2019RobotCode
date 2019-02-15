package frc.subsystem;

import com.ctre.phoenix.motorcontrol.FeedbackDevice;

import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.Timer;
import frc.MotorWrapper;
import frc.PIDController;
import frc.ServiceLocator;
import frc.SolenoidWrapper;
import frc.info.RobotInfo;
import frc.info.SmartDashboardInfo;

public class HatchIntakeSubsystem {
	private MotorWrapper frontIntakeWheel;
	private MotorWrapper groundRollerBallMotor;
	private MotorWrapper groundActuationMotor;
	private Solenoid actuatorHatchSolenoid;
	private PIDController pidController;
	private final SmartDashboardInfo smartDashboardInfo;

	public HatchIntakeSubsystem() {
		ServiceLocator.register(this);

		RobotInfo robotInfo = ServiceLocator.get(RobotInfo.class);
		smartDashboardInfo = ServiceLocator.get(SmartDashboardInfo.class);

		frontIntakeWheel = robotInfo.get(RobotInfo.SWAN);
		groundRollerBallMotor = robotInfo.get(RobotInfo.GROUND_ROLLER_BAR_MOTOR);
		groundActuationMotor = robotInfo.get(RobotInfo.GROUND_ACTUATOR_MOTOR);
		groundActuationMotor.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, 0, 0);
		actuatorHatchSolenoid = new Solenoid(4);
		// actuatorHatchSolenoid = robotInfo.get(RobotInfo.HATCH_ACTUATOR_SOLENOID);
		pidController = new PIDController(1/90, 0, 0);
		pidController.clear(Timer.getFPGATimestamp());
	}

	public void spinInFront() { //spin in front/main intake
		frontIntakeWheel.set(smartDashboardInfo.getNumber(SmartDashboardInfo.HATCH_INTAKE_SPIN_IN_FRONT));
	}
	public void spinOutFront() { //spin out front/main intake
		frontIntakeWheel.set(smartDashboardInfo.getNumber(SmartDashboardInfo.HATCH_INTAKE_SPIN_OUT_FRONT));
	}
	public void spinInBack() { //spin in back/ground intake HAT RIGHT
		groundRollerBallMotor.set(smartDashboardInfo.getNumber(SmartDashboardInfo.HATCH_INTAKE_SPIN_IN_BACK));
 	}
	public void spinOutBack() { //spin out back/ground intake HAT LEFT
		groundRollerBallMotor.set(smartDashboardInfo.getNumber(SmartDashboardInfo.HATCH_INTAKE_SPIN_OUT_BACK));
	}

	public void stopSpinning() { // stops both front and back rolling
		groundRollerBallMotor.set(0);
		frontIntakeWheel.set(0);
	}
	public void setBackIntakeUp() {
		pidController.pid(getGroundIntakeDegrees(), 90);
	}
	public void setBackIntakeDown() {

	}

	 public void setFrontIntakeOut() { //front intake moved out or "down"
	 	actuatorHatchSolenoid.set(true);
	 }

	public void setFrontIntakeIn() { // front intake moved in/retracted
	 	actuatorHatchSolenoid.set(false);
	}

	public double getGroundIntakeDegrees() {
		return (((groundActuationMotor.getSelectedSensorPosition(0) * 360.0) / 1024.0) / 200.0) * 4.0;
	}

	public void teleopPeriodic() {
		pidController.updateTime(Timer.getFPGATimestamp());
	}
}
