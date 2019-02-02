/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.*;
import frc.command.Command;
import frc.subsystem.CargoIntakeSubsystem;
import frc.subsystem.DrivetrainSubsystem;
import frc.subsystem.ElevatorSubsystem;
import frc.subsystem.HatchIntakeSubsystem;
import frc.subsystem.VisionSubsystem;
import frc.info.*;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the TimedRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the build.gradle file in the
 * project.
 * - FRC Team
 222222222222222      1111111   77777777777777777777555555555555555555
2:::::::::::::::22   1::::::1   7::::::::::::::::::75::::::::::::::::5
2::::::222222:::::2 1:::::::1   7::::::::::::::::::75::::::::::::::::5
2222222     2:::::2 111:::::1   777777777777:::::::75:::::555555555555
            2:::::2    1::::1              7::::::7 5:::::5
            2:::::2    1::::1             7::::::7  5:::::5
         2222::::2     1::::1            7::::::7   5:::::5555555555
    22222::::::22      1::::l           7::::::7    5:::::::::::::::5
  22::::::::222        1::::l          7::::::7     555555555555:::::5
 2:::::22222           1::::l         7::::::7                  5:::::5
2:::::2                1::::l        7::::::7                   5:::::5
2:::::2                1::::l       7::::::7        5555555     5:::::5
2:::::2       222222111::::::111   7::::::7         5::::::55555::::::5
2::::::2222222:::::21::::::::::1  7::::::7           55:::::::::::::55
2::::::::::::::::::21::::::::::1 7::::::7              55:::::::::55
2222222222222222222211111111111177777777                 555555555
The Fighting Calculators
*/
public class Robot extends TimedRobot {
	public static final int JOYSTICK_TRIGGER = 1;

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

	private Joystick leftJoystick;
	private Joystick rightJoystick;
	private Joystick gamepad;

	private DrivetrainSubsystem drivetrainSubsystem;
	private HatchIntakeSubsystem hatchIntakeSubsystem;
	private ElevatorSubsystem elevatorSubsystem;
	private CargoIntakeSubsystem cargoIntakeSubsystem;

	// WPI Lib Functions

	/**
	 * This function is run when the robot is first started up and should be used
	 * for any initialization code.
	 */
	@Override
	public void robotInit() {
		hasAutoEnded = false;

		new RobotInfo();

		drivetrainSubsystem = new DrivetrainSubsystem();
		hatchIntakeSubsystem = new HatchIntakeSubsystem();
		elevatorSubsystem = new ElevatorSubsystem();

		leftJoystick = new Joystick(0);
		rightJoystick = new Joystick(1);
		gamepad = new Joystick(2);
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
		hasAutoEnded = false;
	}

	/**
	 * This function is called periodically during autonomous. This is where the
	 * autonomous commands must be executed.
	 *
	 * @see frc.robot.Robot#executeCommand(Command)
	 */
	@Override
	public void autonomousPeriodic() {
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
		 * panel in: left bumper DONE panel out: left trigger DONE panel mechanism out:
		 * back DONE panel mechanism in: start DONE elevator: left stick ALREADY THERE ?
		 * cargo in: right bumper DONE cargo out: right trigger DONE cargo roller out: y
		 * DONE cargo roller in: a DONE floor hatch spin in: hat right DONE ISH, NO HAT
		 * THING floor hatch spin out: hat left DONE ISH, SAME^ floor hatch panel
		 * up/down: right stick ??? preset panel heights for elevator: hold x and move
		 * left stick up/down preset ??? cargo heights for elevator: hold b and move
		 * left stick up/down ???
		 */

		if (gamepad.getRawButton(GAMEPAD_BACK)) {
			if (VisionSubsystem.isTarget()) {
				drivetrainSubsystem.arcadeDrive(0, -Math.signum(VisionSubsystem.getTx()) / 5);
			}
		} else {
			drivetrainSubsystem.blendedDrive(leftJoystick.getY(), -rightJoystick.getX());
		}

		if (gamepad.getRawButton(GAMEPAD_LEFT_BUMPER)) { // left trigger out, left bumper in for hatch intake
			hatchIntakeSubsystem.spinOutFront();
		} else if (gamepad.getRawButton(GAMEPAD_LEFT_TRIGGER)) {
			hatchIntakeSubsystem.spinOutFront();
		} else {
			hatchIntakeSubsystem.stopSpinning();
		}

		if (gamepad.getRawButton(GAMEPAD_Y)) { // top
			elevatorSubsystem.placeElevatorTop();
		}

		if (gamepad.getRawButton(GAMEPAD_X)) { // middle
			elevatorSubsystem.placeElevatorMiddle();
		}

		if (gamepad.getRawButton(GAMEPAD_A)) { // bottom
			elevatorSubsystem.placeElevatorBottom();
		}
		if (gamepad.getRawButton(GAMEPAD_START)) {
			hatchIntakeSubsystem.setFrontIntakeOut();
		}
		if (gamepad.getRawButton(GAMEPAD_BACK)) {
			hatchIntakeSubsystem.setFrontIntakeIn();
		}
		if (gamepad.getRawButton(GAMEPAD_RIGHT_BUMPER)) {
			cargoIntakeSubsystem.rollIn();
		}
		if (gamepad.getRawButton(GAMEPAD_RIGHT_TRIGGER)) {
			cargoIntakeSubsystem.rollOut();
		}
		if (gamepad.getRawButton(GAMEPAD_Y)) {
			cargoIntakeSubsystem.solenoidOut();
		}
		if (gamepad.getRawButton(GAMEPAD_A)) {
			cargoIntakeSubsystem.solenoidIn();
		}
		if (gamepad.getPOV() == POV_RIGHT) { // hat right
			hatchIntakeSubsystem.spinInBack();
		}
		if (gamepad.getPOV() == POV_LEFT) { // hat left
			hatchIntakeSubsystem.spinOutBack();
		}
		elevatorSubsystem.setElevator();
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
	 * <<<<<<< Updated upstream Applies deadband onto input value
	 *
	 * @param value    input for value
	 * @param deadband threshold for deadband
	 * @return value with deadband ======= It applies a deadband to the joystick
	 *         value
	 *
	 * @param value    the input value
	 * @param deadband the threshold
	 * @return deadbanded value >>>>>>> Stashed changes
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
