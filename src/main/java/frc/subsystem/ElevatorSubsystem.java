package frc.subsystem;

import com.ctre.phoenix.motorcontrol.FeedbackDevice;

import edu.wpi.first.wpilibj.Timer;
import frc.MotorWrapper;
import frc.ServiceLocator;
import frc.info.RobotInfo;
import frc.PIDController;

public class ElevatorSubsystem {
    private final MotorWrapper elevatorMotor;
    private PIDController pidController;
    private double pidPreviousTime;
    private double setpoint;

    public ElevatorSubsystem() {
        ServiceLocator.register(this);

        RobotInfo robotInfo = ServiceLocator.get(RobotInfo.class);

        elevatorMotor = robotInfo.get(RobotInfo.ELEVATOR_MOTOR);
        elevatorMotor.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, 0, 0);
    }

    public void manualMove(double motorSpeed) {
        elevatorMotor.set(motorSpeed);
    }

    public void setElevator() {
        double output = pidController.pid(elevatorMotor.getSelectedSensorPosition(0), setpoint);



    }
        
    public void placeElevatorTop() {
        
        if(currentposition < setpoint){
        for(currentposition<setpoint) {
            speed??? = positive movenumber
        }
        if(currentposition > setpoint){
            for(currentposition>setpoint) {
                speed??? = negative movenumber
            }
    }}

    public void placeElevatorMiddle() {
        setpoint = 1;
    }
    
    public void placeElevatorBottom() {
        setpoint = 0;
    }

    public void teleopPeriodic() {
        double dt = Timer.getFPGATimestamp() - pidPreviousTime;
        pidController.updateTime(dt);
    }
}