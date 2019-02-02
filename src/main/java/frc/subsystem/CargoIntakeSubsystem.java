package frc.subsystem;

import frc.MotorWrapper;
import frc.ServiceLocator;
import frc.SolenoidWrapper;
import frc.info.RobotInfo;

public class CargoIntakeSubsystem {
    private final MotorWrapper rollerBarMotor; //big grey
    private final MotorWrapper boxMotor;//small red
    private final SolenoidWrapper rollerBarSolenoid;

    public CargoIntakeSubsystem() {
        ServiceLocator.register(this);
        RobotInfo robotInfo = ServiceLocator.get(RobotInfo.class);
        rollerBarMotor = robotInfo.get(RobotInfo.CARGO_ROLLER_BAR_MOTOR);
        boxMotor = robotInfo.get(RobotInfo.CARGO_BOX_MOTOR);
        rollerBarSolenoid = robotInfo.get(RobotInfo.CARGO_SOLENOID);
    }
    public void rollIn() { //cargo in
        rollerBarMotor.set(1);
        boxMotor.set(.75);
    }
    public void rollOut() { //cargo out
        boxMotor.set(-.5);
        rollerBarMotor.set(-.5);
    }
    public void solenoidOut() { // push out the solenoid, cargo roller out, Y
        rollerBarSolenoid.set(true);
    }
    public void solenoidIn() { // pull in the solenoid, cargo roller in, A
        rollerBarSolenoid.set(false);
    }
}