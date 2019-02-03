package frc.subsystem;

import com.ctre.phoenix.motorcontrol.FeedbackDevice;

import edu.wpi.first.wpilibj.Timer;
import frc.MotorWrapper;
import frc.PIDController;
import frc.SolenoidWrapper;
import frc.ServiceLocator;
import frc.info.RobotInfo;
import frc.info.SmartDashboardInfo;

public class HatchIntakeSubsystem {
	private MotorWrapper frontIntakeWheel;
	private MotorWrapper groundRollerBallMotor;
	private MotorWrapper groundActuationMotor;
	private SolenoidWrapper actuatorHatchSolenoid;
	private PIDController pidController;
	private double pidPreviousTime;
	private final SmartDashboardInfo smartDashboardInfo;

	public HatchIntakeSubsystem() {
		ServiceLocator.register(this);

		RobotInfo robotInfo = ServiceLocator.get(RobotInfo.class);
		smartDashboardInfo = ServiceLocator.get(SmartDashboardInfo.class);

		frontIntakeWheel = robotInfo.get(RobotInfo.HATCH_ROLLER_BAR_MOTOR);
		groundRollerBallMotor = robotInfo.get(RobotInfo.GROUND_ROLLER_BAR_MOTOR);
		groundActuationMotor = robotInfo.get(RobotInfo.GROUND_ACTUATOR_MOTOR);
		groundActuationMotor.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, 0, 0);
		actuatorHatchSolenoid = robotInfo.get(RobotInfo.HATCH_ACTUATOR_SOLENOID);
		pidController = new PIDController(1/90, 0, 0);
		pidPreviousTime = 0;
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
		double dt = Timer.getFPGATimestamp() - pidPreviousTime;
		pidController.updateTime(dt);
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

	public void resetPID() {
		pidController.clear(0);
		pidPreviousTime = Timer.getFPGATimestamp();
	}
}
