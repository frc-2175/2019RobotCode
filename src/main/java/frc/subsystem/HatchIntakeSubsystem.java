package frc.subsystem;

import com.ctre.phoenix.motorcontrol.FeedbackDevice;

import edu.wpi.first.wpilibj.Timer;
import frc.MotorWrapper;
import frc.PIDController;
import frc.SolenoidWrapper;
import frc.ServiceLocator;
import frc.info.RobotInfo;

public class HatchIntakeSubsystem {
	private MotorWrapper rollerBarMotor;
	private MotorWrapper groundRollerBallMotor;
	private MotorWrapper groundActuationMotor;
	private SolenoidWrapper actuatorHatchSolenoid;
	private PIDController pidController;
	private double pidPreviousTime;

	public HatchIntakeSubsystem() {
		ServiceLocator.register(this);

		RobotInfo robotInfo = ServiceLocator.get(RobotInfo.class);

		rollerBarMotor = robotInfo.get(RobotInfo.HATCH_ROLLER_BAR_MOTOR);
		groundRollerBallMotor = robotInfo.get(RobotInfo.GROUND_ROLLER_BAR_MOTOR);
		groundActuationMotor = robotInfo.get(RobotInfo.GROUND_ACTUATOR_MOTOR);
		groundActuationMotor.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, 0, 0);
		actuatorHatchSolenoid = robotInfo.get(RobotInfo.HATCH_ACTUATOR_SOLENOID);
		pidController = new PIDController(1/90, 0, 0);
		pidPreviousTime = 0;
	}

	public void spinIn() {
		rollerBarMotor.set(0.3);
	}
	public void spinInGround() {
		groundRollerBallMotor.set(.3);
 	}
	public void spinOutGround() {
		groundRollerBallMotor.set(-.3);
	}
	public void spinOut() {
		rollerBarMotor.set(-0.3);
	}

	public void stopSpinning() {
		groundRollerBallMotor.set(0);
		rollerBarMotor.set(0);
	}
	public void setGroundIntakeUp() {
		double dt = Timer.getFPGATimestamp() - pidPreviousTime;
		pidController.updateTime(dt);
		pidController.pid(getGroundIntakeDegrees(), 90);
	}
	 public void setHatchIntakeUp() {
	 	actuatorHatchSolenoid.set(true);
	 }

	public void setIntakeDown() {
	 	actuatorHatchSolenoid.set(false);
	}

	public double getGroundIntakeDegrees() {
		return (((groundActuationMotor.getSelectedSensorPosition(0) * 360.0) / 1024.0) / 200.0) * 4.0;
	}

	public void resetPID() {
		pidController.clear();
		pidPreviousTime = Timer.getFPGATimestamp();
	}
}
