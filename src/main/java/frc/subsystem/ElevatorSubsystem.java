package frc.subsystem;

import frc.MotorWrapper;
import frc.ServiceLocator;
import frc.info.RobotInfo;

public class ElevatorSubsystem {
    private final MotorWrapper elevatorMotor;

    public ElevatorSubsystem() {
        ServiceLocator.register(this);
        RobotInfo robotInfo = ServiceLocator.get(RobotInfo.class);
        elevatorMotor = robotInfo.get(RobotInfo.ELEVATOR_MOTOR);
    }

    public void manualMove(double motorSpeed) {
        elevatorMotor.set(motorSpeed);
    }

}
