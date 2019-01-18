package frc.subsystem;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import frc.ServiceLocator;
import frc.info.RobotInfo;

public class CargoIntakeSubsystem {
    private final WPI_TalonSRX rollerBarMotor; //big grey
    private final WPI_TalonSRX boxMotor;//small red

    public CargoIntakeSubsystem() {
        ServiceLocator.register(this);
        RobotInfo robotInfo = ServiceLocator.get(RobotInfo.class);
        rollerBarMotor = robotInfo.get(RobotInfo.CARGO_ROLLER_BAR_MOTOR);
        boxMotor = robotInfo.get(RobotInfo.CARGO_BOX_MOTOR);
    }
    public void rollIn() {
        rollerBarMotor.set(1);
        boxMotor.set(.75);
    }
    public void rollOutElevator() { //roll it out fast to shoot out the bal !!!
        boxMotor.set(-1);
    }
    public void rollOutBottom() { //roll it out softly if it's on the ground or something
        boxMotor.set(-.5);
        rollerBarMotor.set(-.5);
    }
}