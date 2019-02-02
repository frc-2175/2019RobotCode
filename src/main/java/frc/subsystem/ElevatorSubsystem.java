package frc.subsystem;

import com.ctre.phoenix.motorcontrol.FeedbackDevice;

import edu.wpi.first.wpilibj.Timer;
import frc.MotorWrapper;
import frc.ServiceLocator;
import frc.info.RobotInfo;
import frc.PIDController;

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
    private final MotorWrapper elevatorMotor;
    private PIDController pidController;
    private double pidPreviousTime;
	private double setpoint;
	private boolean isManual; //

    public ElevatorSubsystem() {
        ServiceLocator.register(this);

        RobotInfo robotInfo = ServiceLocator.get(RobotInfo.class);

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
        setpoint = 2;
    }

    public void CargoPlaceElevatorMiddle() {
        setpoint = 1;
    }

    public void CargoPlaceElevatorBottom() {
        setpoint = 0;
	}
	public void HatchPlaceElevatorTop() {
        setpoint = 2;
    }

    public void HatchPlaceElevatorMiddle() {
        setpoint = 1;
    }

    public void HatchPlaceElevatorBottom() {
        setpoint = 0;
    }

    public void teleopPeriodic() {
        double dt = Timer.getFPGATimestamp() - pidPreviousTime;
        pidController.updateTime(dt);
    }
}
