package frc.subsystem;

import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.MotorWrapper;
import frc.PIDController;
import frc.ServiceLocator;
import frc.info.RobotInfo;
import frc.info.SmartDashboardInfo;

/*
goToBottomPanel
goToBottomCargo
goToMiddlePanel
goToTopPanel
goToMiddleCargo
goToTopCargo

setMode(automatic or manual)

manualMove
*/

public class ElevatorSubsystem {
	private final SmartDashboardInfo smartDashboardInfo;
	private final MotorWrapper elevatorMotor;
	private final MotorWrapper elevatorMotorFollower;
    private PIDController pidController;
	private double setpoint;
	private boolean isManual;
	private final double elevatorKP;
	private final double elevatorKI;
	private final double elevatorKD;

    public ElevatorSubsystem() {
        ServiceLocator.register(this);

		RobotInfo robotInfo = ServiceLocator.get(RobotInfo.class);
		smartDashboardInfo = ServiceLocator.get(SmartDashboardInfo.class);

		elevatorMotor = robotInfo.get(RobotInfo.ELEVATOR_MOTOR_MASTER);
		elevatorMotorFollower = robotInfo.get(RobotInfo.ELEVATOR_MOTOR_FOLLOWER);
		elevatorMotorFollower.follow(elevatorMotor);
		elevatorMotor.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, 0, 0);
		elevatorMotor.setSelectedSensorPosition(0, 0, 0);

		elevatorKP = smartDashboardInfo.getNumber(smartDashboardInfo.ELEVATOR_PID_P);
		elevatorKI = smartDashboardInfo.getNumber(smartDashboardInfo.ELEVATOR_PID_I);
		elevatorKD = smartDashboardInfo.getNumber(smartDashboardInfo.ELEVATOR_PID_D);
		pidController = new PIDController(elevatorKP, elevatorKI, elevatorKD);
	}
	public void setIsManual(boolean x) {
		isManual = x;
	}

    public void manualMove(double motorSpeed) {
		if(isManual) {
			elevatorMotor.set(motorSpeed);
		}
    }


	public void setElevator() {
		if (!isManual) {
			double output = pidController.pid(getElevatorPosition(), setpoint); //what to set motor speed to
			elevatorMotor.set(output); //setting motor speed to speed needed to go to setpoint
			SmartDashboard.putNumber("AutoPopulate/ElevatorOutput", output);
		}
	}

    public void CargoPlaceElevatorTop() {
        setpoint = smartDashboardInfo.getNumber(SmartDashboardInfo.CARGO_TOP_SETPOINT);
    }

    public void CargoPlaceElevatorMiddle() {
        setpoint = smartDashboardInfo.getNumber(SmartDashboardInfo.CARGO_MIDDLE_SETPOINT);
    }

    public void CargoPlaceElevatorBottom() {
        setpoint = smartDashboardInfo.getNumber(SmartDashboardInfo.CARGO_BOTTOM_SETPOINT);
	}
	public void HatchPlaceElevatorTop() {
        setpoint = smartDashboardInfo.getNumber(SmartDashboardInfo.HATCH_TOP_SETPOINT);
    }

    public void HatchPlaceElevatorMiddle() {
        setpoint = smartDashboardInfo.getNumber(SmartDashboardInfo.HATCH_MIDDLE_SETPOINT);
    }

    public void HatchPlaceElevatorBottom() {
        setpoint = smartDashboardInfo.getNumber(SmartDashboardInfo.HATCH_BOTTOM_SETPOINT);
    }

    public void teleopPeriodic() {
        pidController.updateTime(Timer.getFPGATimestamp());
	}

	public double getCurrentDraw() {
		if(elevatorMotor.isTalon) {
			return ((WPI_TalonSRX)(elevatorMotor.getMotor())).getOutputCurrent();
		} else {
			return 0;
		}
	}

	public double getElevatorPosition() {
		return 3 * -elevatorMotor.getSelectedSensorPosition(0) * 1.273 * Math.PI / 4096;
	}

	public void zeroEncoder() {
		elevatorMotor.setSelectedSensorPosition(0, 0, 0);
	}

	public boolean getIsManual() {
		return isManual;
	}
}
