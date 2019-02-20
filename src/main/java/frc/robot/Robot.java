/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.PIDController;
import frc.Vector;
import frc.command.Command;
import frc.info.RobotInfo;
import frc.info.SmartDashboardInfo;
import frc.subsystem.CargoIntakeSubsystem;
import frc.subsystem.DrivetrainSubsystem;
import frc.subsystem.ElevatorSubsystem;
import frc.subsystem.HatchIntakeSubsystem;
import frc.subsystem.VisionSubsystem;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the TimedRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the build.gradle file in the
 * project. - FRC Team
 * 222222222222222       1111111     77777777777777777777 555555555555555555
 * 2:::::::::::::::22    1::::::1    7::::::::::::::::::7 5::::::::::::::::5
 * 2::::::222222:::::2  1:::::::1    7::::::::::::::::::7 5::::::::::::::::5
 * 2222222     2:::::2  111:::::1    777777777777:::::::7 5:::::555555555555
 *             2:::::2     1::::1               7::::::7  5:::::5
 *             2:::::2     1::::1              7::::::7   5:::::5
 *          2222::::2      1::::1             7::::::7    5:::::5555555555
 *     22222::::::22       1::::l            7::::::7     5:::::::::::::::5
 *   22::::::::222         1::::l           7::::::7      555555555555:::::5
 *  2:::::22222            1::::l          7::::::7                   5:::::5
 * 2:::::2                 1::::l         7::::::7                    5:::::5
 * 2:::::2                 1::::l        7::::::7         5555555     5:::::5
 * 2:::::2       222222 111::::::111    7::::::7          5::::::55555::::::5
 * 2::::::2222222:::::2 1::::::::::1   7::::::7            55:::::::::::::55
 * 2::::::::::::::::::2 1::::::::::1  7::::::7               55:::::::::55
 * 22222222222222222222 111111111111 77777777                  555555555
 * The Fighting Calculators
 */
public class Robot extends TimedRobot {
	public static final int JOYSTICK_TRIGGER = 1;

	public double previousJoystickValue = 0;
	public static final double TOPLINE = .6;
	public static final double BOTTOMLINE = -.6;

	public static final int GAMEPAD_X = 1;
	public static final int GAMEPAD_A = 2;
	public static final int GAMEPAD_B = 3;
	public static final int GAMEPAD_Y = 4;
	public static final int GAMEPAD_LEFT_BUMPER = 5;
	public static final int GAMEPAD_RIGHT_BUMPER = 6;
	public static final int GAMEPAD_LEFT_TRIGGER = 7;
	public static final int GAMEPAD_RIGHT_TRIGGER = 8;
	public static final int GAMEPAD_BACK = 9;
	public static final int GAMEPAD_START = 10;
	public static final int GAMEPAD_LEFT_STICK_PRESS = 11;
	public static final int GAMEPAD_RIGHT_STICK_PRESS = 12;

	public static final int POV_UP = 0;
	public static final int POV_UP_RIGHT = 45;
	public static final int POV_RIGHT = 90;
	public static final int POV_DOWN_RIGHT = 135;
	public static final int POV_DOWN = 180;
	public static final int POV_DOWN_LEFT = 225;
	public static final int POV_LEFT = 270;
	public static final int POV_UP_LEFT = 315;

	private boolean hasAutoEnded;
	private boolean isPreviousManual;

	private Joystick leftJoystick;
	private Joystick rightJoystick;
	private Joystick gamepad;

	private DrivetrainSubsystem drivetrainSubsystem;
	private HatchIntakeSubsystem hatchIntakeSubsystem;
	private ElevatorSubsystem elevatorSubsystem;
	private CargoIntakeSubsystem cargoIntakeSubsystem;
	private VisionSubsystem visionSubsystem;
	private SmartDashboardInfo smartDashboardInfo;

	// WPI Lib Functions

	/**
	 * This function is run when the robot is first started up and should be used
	 * for any initialization code.
	 */
	@Override
	public void robotInit() {
		hasAutoEnded = false;

		new RobotInfo();
		smartDashboardInfo = new SmartDashboardInfo();

		visionSubsystem = new VisionSubsystem();
		drivetrainSubsystem = new DrivetrainSubsystem();
		hatchIntakeSubsystem = new HatchIntakeSubsystem();
		elevatorSubsystem = new ElevatorSubsystem();
		cargoIntakeSubsystem = new CargoIntakeSubsystem();

		leftJoystick = new Joystick(0);
		rightJoystick = new Joystick(1);
		gamepad = new Joystick(2);

		// new Thread(() -> {
		// 	UsbCamera camera = CameraServer.getInstance().startAutomaticCapture();
		// 	camera.setResolution(320, 240);
			
		// 	CvSink cvSink = CameraServer.getInstance().getVideo();
		// 	CvSource outputStream = CameraServer.getInstance().putVideo("FlippedCamera", 320, 240);
			
		// 	Mat source = new Mat();
		// 	Mat output = new Mat();
			
		// 	while(!Thread.interrupted()) {
		// 		cvSink.grabFrame(source);
		// 		Core.rotate(source, output, Core.ROTATE_180);
		// 		outputStream.putFrame(output);
		// 	}
		// }).start();
	}

	/**
	 * This function is called every robot packet, no matter the mode. Use this for
	 * items like diagnostics that you want ran during disabled, autonomous,
	 * teleoperated and test.
	 *
	 * <p>
	 * This runs after the mode specific periodic functions, but before LiveWindow
	 * and SmartDashboard integrated updating.
	 */
	@Override
	public void robotPeriodic() {
	}

	@Override
	public void disabledInit() {
		System.out.println("Robot program is disabled and ready.");
	}

	/**
	 * This autonomous (along with the chooser code above) shows how to select
	 * between different autonomous modes using the dashboard. The sendable chooser
	 * code works with the Java SmartDashboard. If you prefer the LabVIEW Dashboard,
	 * remove all of the chooser code and uncomment the getString line to get the
	 * auto name from the text box below the Gyro
	 *
	 * <p>
	 * You will need to initialize whatever autonomous command is selected inside
	 * this method.
	 */
	@Override
	public void autonomousInit() {
		// hasAutoEnded = false;
	}

	/**
	 * This function is called periodically during autonomous. This is where the
	 * autonomous commands must be executed.
	 *
	 * @see #executeCommand(Command)
	 */
	@Override
	public void autonomousPeriodic() {
		teleopPeriodic();
	}

	@Override
	public void teleopInit() {
		elevatorSubsystem.zeroEncoder();
	}

	/**
	 * This function is called periodically during operator control.
	 */
	@Override
	public void teleopPeriodic() {
		/*
		 * Alayna's controls:
		 *
		 * manual driving: left and right joysticks automatic driving: right trigger
		 *
		 * panel in: left bumper DONE
		 * panel out: left trigger DONE
		 * panel mechanism out:back DONE
		 * panel mechanism in: start DONE
		 * cargo in: right bumper DONE
		 * cargo out: right trigger DONE
		 * cargo roller out: y DONE
		 * cargo roller in: a DONE
		 * floor hatch spin in: hat right DONE I THINK
		 * floor hatch spin out: hat left DONE I THINK
		 * floor hatch panel up/down: right stick DONE
		 * preset panel heights for elevator: hold x and move left stick up/down DONE
		 * preset cargo heights for elevator: hold b and move DONE
		 * left stick up/down DONE I THINK
		 */

		SmartDashboard.putNumberArray("Cargo Setpoints", elevatorSubsystem.getCargoSetpoints());
		SmartDashboard.putNumberArray("Hatch Setpoints", elevatorSubsystem.getHatchSetpoints());

		// Driving
		if(rightJoystick.getRawButtonPressed(2)) {
			drivetrainSubsystem.storeTargetHeading();
		}
		if(rightJoystick.getRawButton(2) && visionSubsystem.doesValidTargetExist()) {
			drivetrainSubsystem.driveWithSimpleVision(-leftJoystick.getY());
		} else {
			drivetrainSubsystem.blendedDrive(-leftJoystick.getY(), rightJoystick.getX());
		}

		if (gamepad.getRawButton(GAMEPAD_LEFT_BUMPER) || leftJoystick.getRawButton(1)) { // left trigger out, left bumper in for hatch intake
			hatchIntakeSubsystem.spinOutFront();
		} else if (gamepad.getRawButton(GAMEPAD_LEFT_TRIGGER)) {
			hatchIntakeSubsystem.spinInFront();
		} else {
			hatchIntakeSubsystem.stopSpinning();
		}

		if (gamepad.getRawButton(GAMEPAD_START)) {
			hatchIntakeSubsystem.setFrontIntakeOut();
		}
		if (gamepad.getRawButton(GAMEPAD_BACK)) {
			hatchIntakeSubsystem.setFrontIntakeIn();
		}
		if (gamepad.getRawButton(GAMEPAD_RIGHT_BUMPER) || rightJoystick.getRawButton(1)) {
			cargoIntakeSubsystem.rollOut();
		} else if (gamepad.getRawButton(GAMEPAD_RIGHT_TRIGGER)) {
			cargoIntakeSubsystem.rollIn();
		} else {
			cargoIntakeSubsystem.stopAllMotors();
		}
		if(gamepad.getRawButtonPressed(GAMEPAD_RIGHT_TRIGGER)) {
			cargoIntakeSubsystem.solenoidOut();
		}
		if(gamepad.getRawButtonReleased(GAMEPAD_RIGHT_TRIGGER)) {
			cargoIntakeSubsystem.solenoidIn();
		}
		if (gamepad.getRawButton(GAMEPAD_Y)) {
			cargoIntakeSubsystem.solenoidIn();
		}
		if (gamepad.getRawButton(GAMEPAD_A)) {
			cargoIntakeSubsystem.solenoidOut();
		}
		if (gamepad.getPOV() == POV_RIGHT) { // hat right
			hatchIntakeSubsystem.spinInBack();
		}
		if (gamepad.getPOV() == POV_LEFT) { // hat left
			hatchIntakeSubsystem.spinOutBack();
		}
		boolean isManual = !(gamepad.getRawButton(GAMEPAD_X) || gamepad.getRawButton(GAMEPAD_B));
		elevatorSubsystem.setIsManual(isManual);
		if (!isManual && isPreviousManual) {
			elevatorSubsystem.setSetpoint(elevatorSubsystem.getElevatorPosition());
		}
		if ((-gamepad.getRawAxis(1) > TOPLINE && previousJoystickValue <= TOPLINE) || (-gamepad.getRawAxis(1) < BOTTOMLINE && previousJoystickValue >= BOTTOMLINE)) { //if you flicked either way
			elevatorSubsystem.setStickMoved(true); //say the stick moved
			double[] setpoints = gamepad.getRawButton(GAMEPAD_X) ? elevatorSubsystem.getHatchSetpoints() : elevatorSubsystem.getCargoSetpoints();
			double preset = elevatorSubsystem.getElevatorPreset(setpoints, -gamepad.getRawAxis(1) > 0);
			if(preset != -1) {
				elevatorSubsystem.setSetpoint(preset);
			}
		}
		/* if(gamepad.getPOV() == POV_UP) {
			hatchIntakeSubsystem.setIsManual(true);
		} else {
			hatchIntakeSubsystem.setIsManual(false);
			if (-gamepad.getRawAxis(3) > TOPLINE) {
				hatchIntakeSubsystem.setBackIntakeUp();
			} else if (-gamepad.getRawAxis(3) < BOTTOMLINE) {
				hatchIntakeSubsystem.setBackIntakeDown();
			}
		} */
		double elevatorSpeed = deadband(-gamepad.getRawAxis(1), 0.05) >= 0 ? deadband(-gamepad.getRawAxis(1), 0.05) * 0.8 : deadband(-gamepad.getRawAxis(1), 0.05) * 0.5;
		if(isManual) {
			elevatorSubsystem.manualMove(elevatorSpeed);
		} else {
			elevatorSubsystem.setElevator();
		}
		if(elevatorSpeed > 0.05) {
			cargoIntakeSubsystem.spinRollerbarForElevator();
		}

		hatchIntakeSubsystem.setBackIntakeSpeed(-gamepad.getRawAxis(3) * 0.5);
		if(leftJoystick.getRawButton(2)) {
			hatchIntakeSubsystem.setBackIntakeSpeed(-0.5);
			hatchIntakeSubsystem.spinOutBack();
		}
		// hatchIntakeSubsystem.goToSetpoint();
		previousJoystickValue = -gamepad.getRawAxis(1);
		SmartDashboard.putNumber("AutoPopulate/ElevatorCurrentDraw", elevatorSubsystem.getCurrentDraw());
		SmartDashboard.putNumber("AutoPopulate/ElevatorPosition", elevatorSubsystem.getElevatorPosition());
		SmartDashboard.putBoolean("AutoPopulate/IsManual", elevatorSubsystem.getIsManual());
		isPreviousManual = isManual;
		hatchIntakeSubsystem.teleopPeriodic();
		elevatorSubsystem.teleopPeriodic();
		drivetrainSubsystem.teleopPeriodic();
	}

	/**
	 * This function is called periodically during test mode.
	 */
	@Override
	public void testPeriodic() {
	}

	// Custom Functions

	/**
	 * Runs the execute portion of a command until it is finished. When it is
	 * finished, it'll call the end portion of the command once.
	 * <p>
	 * Note: this method will not call the initialize portion of the command.
	 *
	 * @param command the command to execute
	 */
	public void executeCommand(Command command) {
		if (!hasAutoEnded) {
			if (!command.isFinished()) {
				command.execute();
			} else {
				command.end();
				hasAutoEnded = true;
			}
		}
	}

	/**
	 * Applies a deadband with ramping onto an input value
	 * @param value input for value
	 * @param deadband threshold for deadband
	 * @return value with deadband
	 */
	public static double deadband(double value, double deadband) {
		if (Math.abs(value) > deadband) {
			if (value > 0.0) {
				return (value - deadband) / (1.0 - deadband);
			} else {
				return (value + deadband) / (1.0 - deadband);
			}
		} else {
			return 0.0;
		}
	}
}
