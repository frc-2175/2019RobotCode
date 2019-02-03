package frc.subsystem;

import com.ctre.phoenix.motorcontrol.FeedbackDevice;

import edu.wpi.first.wpilibj.Timer;
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
    private PIDController pidController;
    private double pidPreviousTime;
	private double setpoint;
	private boolean isManual; //

    public ElevatorSubsystem() {
        ServiceLocator.register(this);

		RobotInfo robotInfo = ServiceLocator.get(RobotInfo.class);
		smartDashboardInfo = ServiceLocator.get(SmartDashboardInfo.class);

        elevatorMotor = robotInfo.get(RobotInfo.ELEVATOR_MOTOR);
        elevatorMotor.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, 0, 0);
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
        double output = pidController.pid(elevatorMotor.getSelectedSensorPosition(0), setpoint); //what to set motor speed to
        elevatorMotor.set(output); //setting motor speed to speed needed to go to setpoint
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
        double dt = Timer.getFPGATimestamp() - pidPreviousTime;
        pidController.updateTime(dt);
    }
}
