package frc.subsystem;

import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import frc.info.*;
import frc.ServiceLocator;

public class CargoIntakeSubsystem {
    private final TalonSRX rollerBarMotor; //big grey
    private final TalonSRX boxMotor;//small red

    public CargoIntakeSubsystem() {
        RobotInfo robotInfo = ServiceLocator.get(RobotInfo.class);
        rollerBarMotor = robotInfo.get(RobotInfo.CARGO_ROLLER_BAR_MOTOR);
        boxMotor = robotInfo.get(RobotInfo.CARGO_BOX_MOTOR);
    }
}