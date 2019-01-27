package frc.subsystem;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import frc.ServiceLocator;

import frc.info.RobotInfo;

public class ElevatorSubsystem {
    private final WPI_TalonSRX  elevatorMotor;

    public ElevatorSubsystem() {
        ServiceLocator.register(this);
        RobotInfo robotInfo = ServiceLocator.get(RobotInfo.class);
        elevatorMotor = robotInfo.get(RobotInfo.ELEVATOR_MOTOR);
    }

    public void manualMove(double motorSpeed) {
        elevatorMotor.set(motorSpeed);
    }

}
