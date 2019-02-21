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
	private boolean stickMoved;
	private final double elevatorKP;
	private final double elevatorKI;
	private final double elevatorKD;
	private final double setpointThreshold;
	private double[] cargoSetpoints = {16, 44, 72};
	private double[] hatchSetpoints = {28, 56};


    public ElevatorSubsystem() {
        ServiceLocator.register(this);

		RobotInfo robotInfo = ServiceLocator.get(RobotInfo.class);
		smartDashboardInfo = ServiceLocator.get(SmartDashboardInfo.class);

		elevatorMotor = robotInfo.get(RobotInfo.ELEVATOR_MOTOR_MASTER);
		elevatorMotorFollower = robotInfo.get(RobotInfo.ELEVATOR_MOTOR_FOLLOWER);
		elevatorMotorFollower.follow(elevatorMotor);
		elevatorMotor.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, 0, 0);
		elevatorMotor.setSelectedSensorPosition(0, 0, 0);
		zeroEncoder();

		setpointThreshold = 6;
		elevatorKP = smartDashboardInfo.getNumber(SmartDashboardInfo.ELEVATOR_PID_P);
		elevatorKI = smartDashboardInfo.getNumber(SmartDashboardInfo.ELEVATOR_PID_I);
		elevatorKD = smartDashboardInfo.getNumber(SmartDashboardInfo.ELEVATOR_PID_D);
		pidController = new PIDController(elevatorKP, elevatorKI, elevatorKD);

	}
	public void setIsManual(boolean x) {
		isManual = x;
	}
	public void setStickMoved(boolean x) {
		stickMoved = x;
	}

    public void manualMove(double motorSpeed) {
		if(isManual) {
			elevatorMotor.set(motorSpeed);
		}
    }

	public void setElevator() {
		if (!isManual) {
			SmartDashboard.putNumber("setpoint", setpoint);
			double output = pidController.pid(getElevatorPosition(), setpoint, 4); //what to set motor speed to
			output += 0.125;
			output = clamp(output, -0.4, 0.5);
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

	public static double clamp(double val, double min, double max) {
		return val >= min && val <= max ? val : (val < min ? min : max);
	}
/*
	public void runCargoElevatorPreset(double axisValue) {
		double elevatorPosition = getElevatorPosition();
		if(axisValue > 0) { //if stick is up
			if(elevatorPosition < (cargoBottomSetpoint - setpointThreshold)) { //quadrant 1 up
				CargoPlaceElevatorBottom();
			} else if((cargoMiddleSetpoint - setpointThreshold) >= elevatorPosition && elevatorPosition > (cargoBottomSetpoint - setpointThreshold)) { //quadrant 2 up
				CargoPlaceElevatorMiddle();
			} else if((cargoTopSetpoint - setpointThreshold) >= elevatorPosition && elevatorPosition > (cargoMiddleSetpoint - setpointThreshold)) { //quadrant 3 up
				CargoPlaceElevatorTop();
			} else { //quadrant 4 up
				CargoPlaceElevatorTop();
			}
		} else { //if stick is down
			if(elevatorPosition > (cargoTopSetpoint + setpointThreshold)) { //quadrant 4 down
				CargoPlaceElevatorTop();
			} else if((cargoTopSetpoint + setpointThreshold) >= elevatorPosition && elevatorPosition > (cargoMiddleSetpoint + setpointThreshold)) { //quadrant 3 down
				CargoPlaceElevatorMiddle();
			} else if((cargoMiddleSetpoint + setpointThreshold) >= elevatorPosition && elevatorPosition > (cargoBottomSetpoint + setpointThreshold)) { // quadrant 2 down
				CargoPlaceElevatorBottom();
			} else { //quadrant 1 down
				CargoPlaceElevatorBottom();
			}
		}
	}
	*/

	public double getElevatorPreset(double[] setpoints, boolean isUp) {
		double elevatorPosition = getElevatorPosition();
		if(isUp) {
			for(double point : setpoints) {
				if(point - elevatorPosition > setpointThreshold) {
					return point;
					// setpoint = point;
				}
			}
		} else {
			for(int i = (setpoints.length - 1); i >= 0; i--) {
				if(setpoints[i] - elevatorPosition < -setpointThreshold) {
					return setpoints[i];
					// setpoint = setpoints[i];
				}
			}
		}
		return -1;
	}

	public void setSetpoint(double inputPoint) {
		setpoint = inputPoint;
	}

	public double[] getCargoSetpoints() {
		return cargoSetpoints;
	}

	public double[] getHatchSetpoints() {
		return hatchSetpoints;
	}

	public boolean getIsElevatorAtBottom() {
		// TODO: this value is arbitrary and should eventually be a SmartDashboard value
		return getElevatorPosition() < 8;
	}
}
