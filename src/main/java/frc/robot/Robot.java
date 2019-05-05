/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.Bezier;
import frc.Vector;
import frc.command.Command;
import frc.info.RobotInfo;
import frc.info.SmartDashboardInfo;
import frc.subsystem.CargoIntakeSubsystem;
import frc.subsystem.ClimbingSubsystem;
import frc.subsystem.DrivetrainSubsystem;
import frc.subsystem.ElevatorSubsystem;
import frc.subsystem.HatchIntakeSubsystem;
import frc.subsystem.VisionSubsystem;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the TimedRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the build.gradle file in the
 * project. - FRC Team 222222222222222 1111111 77777777777777777777
 * 555555555555555555 2:::::::::::::::22 1::::::1 7::::::::::::::::::7
 * 5::::::::::::::::5 2::::::222222:::::2 1:::::::1 7::::::::::::::::::7
 * 5::::::::::::::::5 2222222 2:::::2 111:::::1 777777777777:::::::7
 * 5:::::555555555555 2:::::2 1::::1 7::::::7 5:::::5 2:::::2 1::::1 7::::::7
 * 5:::::5 2222::::2 1::::1 7::::::7 5:::::5555555555 22222::::::22 1::::l
 * 7::::::7 5:::::::::::::::5 22::::::::222 1::::l 7::::::7 555555555555:::::5
 * 2:::::22222 1::::l 7::::::7 5:::::5 2:::::2 1::::l 7::::::7 5:::::5 2:::::2
 * 1::::l 7::::::7 5555555 5:::::5 2:::::2 222222 111::::::111 7::::::7
 * 5::::::55555::::::5 2::::::2222222:::::2 1::::::::::1 7::::::7
 * 55:::::::::::::55 2::::::::::::::::::2 1::::::::::1 7::::::7 55:::::::::55
 * 22222222222222222222 111111111111 77777777 555555555 The Fighting Calculators
 */
public class Robot extends TimedRobot {
	public static final int JOYSTICK_TRIGGER = 1;

	public double previousJoystick1Value = 0;
	public double previousJoystick3Value = 0;
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
	public static final double extraSpace = 5.0;
	public static boolean isGoingCargoShip = false;

	private boolean hasAutoEnded;
	private boolean isPreviousManual;
	private boolean stayingAutomatic;

	private Joystick leftJoystick;
	private Joystick rightJoystick;
	private Joystick gamepad;

	private DrivetrainSubsystem drivetrainSubsystem;
	private ClimbingSubsystem climbingSubsystem;
	private HatchIntakeSubsystem hatchIntakeSubsystem;
	private ElevatorSubsystem elevatorSubsystem;
	private CargoIntakeSubsystem cargoIntakeSubsystem;
	private VisionSubsystem visionSubsystem;
	private SmartDashboardInfo smartDashboardInfo;
	private Vector[] autonPath;
	private Vector targetLocation = new Vector(0, 0);
	Vector[] path;

	// WPI Lib Functions

	/**
	 * This function is run when the robot is first started up and should be used
	 * for any initialization code.
	 */
	@Override
	public void robotInit() {
		hasAutoEnded = false; // TODO: Should we remove this variable since we're not even doing any auto?

		new RobotInfo();
		smartDashboardInfo = new SmartDashboardInfo();

		visionSubsystem = new VisionSubsystem();
		drivetrainSubsystem = new DrivetrainSubsystem();
		hatchIntakeSubsystem = new HatchIntakeSubsystem();
		elevatorSubsystem = new ElevatorSubsystem();
		cargoIntakeSubsystem = new CargoIntakeSubsystem();
		climbingSubsystem = new ClimbingSubsystem();

		leftJoystick = new Joystick(0);
		rightJoystick = new Joystick(1);
		gamepad = new Joystick(2);

		Vector[] pathThing = Bezier.getSamplePath();
		double[] xcoordinates = new double[pathThing.length];
		double[] ycoordinates = new double[pathThing.length];
		for(int i = 0; i < pathThing.length; i++) {
			xcoordinates[i] = pathThing[i].x;
			ycoordinates[i] = pathThing[i].y;
		}
		path = new Vector[30];

		SmartDashboard.putNumberArray("Values/PathXCoords", xcoordinates);
		SmartDashboard.putNumberArray("Values/PathYCoords", ycoordinates);
		if(SmartDashboard.putString("CameraToggle/CameraSelected", "1")) {
			System.out.println("Worked");
		} else {
			System.out.println("Didn't Work");
		}
		autonPath = Bezier.getSamplePath();
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
		stayingAutomatic = false;
		drivetrainSubsystem.resetTracking();
		SmartDashboard.putString("CameraToggle/CameraSelected", "0");
		drivetrainSubsystem.resetTracking();
		elevatorSubsystem.zeroEncoder();
	}

	/**
	 * This function is called periodically during autonomous. This is where the
	 * autonomous commands must be executed.
	 *
	 * @see #executeCommand(Command)
	 */
	@Override
	public void autonomousPeriodic() {
		SmartDashboard.putNumber("Values/PositionX", drivetrainSubsystem.fieldPosition.x);
		SmartDashboard.putNumber("Values/PositionY", drivetrainSubsystem.fieldPosition.y);
		teleopPeriodic();
	}

	@Override
	public void disabledPeriodic() {
	}

	@Override
	public void teleopInit() {
		SmartDashboard.putString("CameraToggle/CameraSelected", "0");
	}

	/**
	 * This function is called periodically during operator control.
	 */
	@Override
	public void teleopPeriodic() {
		/**
		 * Full controls list (update if you can while changing things)
		 * Left Joystick
		 * 	- Button 1 (Trigger): front hatch panel outtake
		 * 	- Button 2 (Center Face Button): floor hatch panel outtake and actuate down
		 * Right Joystick
		 * 	- Button 1 (Trigger): outtake cargo
		 * 	- Button 2 (Center Face Button): simple vision for the cargo goal
		 * Gamepad
		 * 	- X Button (while held): hatch panel presets for the elevator (flick left stick up for up one preset,
		 * 		down for down a preset)
		 * 	- A Button: actuate cargo intake out
		 * 	- B Button (while held): cargo presets for the elevator (flick left stick up for up one preset,
		 * 		down for down a preset)
		 * 	- Y Button: actuate cargo intake in
		 * 	- Left Bumper: outtake front hatch panel
		 * 	- Right Bumber: outtake cargo
		 * 	- Left Trigger: intake hatch panels front
		 * 	- Right Trigger: intake cargo with actuation
		 * 	- Start: toggle hatch panel intake front
		 * 	- Left Stick: control elevator
		 * 	- Right Stick: control floor hatch intake
		 * 	- D-pad Right: spin floor intake in
		 * 	- D-pad Left: spin floor intake out
		 */


		SmartDashboard.putNumberArray("Cargo Setpoints", elevatorSubsystem.getCargoSetpoints());
		SmartDashboard.putNumberArray("Hatch Setpoints", elevatorSubsystem.getHatchSetpoints());

		drivetrainSubsystem.trackLocation();
		SmartDashboard.putNumber("Values/PositionX", drivetrainSubsystem.fieldPosition.x);
		SmartDashboard.putNumber("Values/PositionY", drivetrainSubsystem.fieldPosition.y);
		SmartDashboard.putNumber("Values/LeftSideEncoder", drivetrainSubsystem.getLeftSideDistanceDriven());
		SmartDashboard.putNumber("Values/RightSideEncoder", drivetrainSubsystem.getRightSideDistanceDriven());

		// Driving

		if(leftJoystick.getRawButtonPressed(3)) {
			drivetrainSubsystem.storeTargetHeading();
		}
		if(leftJoystick.getRawButton(3)) {
			drivetrainSubsystem.steerTowardVisionTarget(-leftJoystick.getY(), rightJoystick.getX() * 0.25);
		} else {
			drivetrainSubsystem.blendedDrive(-leftJoystick.getY(), rightJoystick.getX());
		}


		if(rightJoystick.getRawButtonPressed(3)) {
			String value = SmartDashboard.getString("CameraToggle/CameraSelected", "0").equals("0") ? "1" : "0";
			SmartDashboard.putString("CameraToggle/CameraSelected", value);
		}

		// Front Hatch Intake

		// Intaking
		if (gamepad.getRawButton(GAMEPAD_LEFT_BUMPER) || leftJoystick.getRawButton(1)) { // left trigger out, left bumper in for hatch intake
			hatchIntakeSubsystem.spinOutFront();
		} else if (gamepad.getRawButton(GAMEPAD_LEFT_TRIGGER)) {
			hatchIntakeSubsystem.spinInFront();
		} else {
			hatchIntakeSubsystem.stopSpinning();
		}
		// Actuation
		if(gamepad.getRawButtonPressed(GAMEPAD_START)) {
			hatchIntakeSubsystem.toggleFrontIntake();
		}

		// Cargo Intake

		// Intaking/Outtaking
		if (gamepad.getRawButton(GAMEPAD_RIGHT_BUMPER) || rightJoystick.getRawButton(1)) {
			if(elevatorSubsystem.getIsElevatorAtBottom() || gamepad.getRawButton(GAMEPAD_RIGHT_BUMPER)) {
				cargoIntakeSubsystem.rollOut();
			} else {
				cargoIntakeSubsystem.rollJustBoxOut();
			}
		} else if (gamepad.getRawButton(GAMEPAD_RIGHT_TRIGGER)) {
			cargoIntakeSubsystem.rollIn();
		} else {
			cargoIntakeSubsystem.stopAllMotors();
		}
		// Actuation when pressed/released (only when elevator is near bottom as to avoid punching the rocket)
		if(elevatorSubsystem.getIsElevatorAtBottom()) {
			if(gamepad.getRawButtonPressed(GAMEPAD_RIGHT_TRIGGER)) {
				cargoIntakeSubsystem.solenoidOut();
			}
			if(gamepad.getRawButtonReleased(GAMEPAD_RIGHT_TRIGGER)) {
				cargoIntakeSubsystem.solenoidIn();
			}
		}
		// Manual actuation on buttons
		if (gamepad.getRawButton(GAMEPAD_Y)) {
			cargoIntakeSubsystem.solenoidIn();
		}
		if (gamepad.getRawButton(GAMEPAD_A)) {
			cargoIntakeSubsystem.solenoidOut();
		}

		// Elevator

		boolean isManual = !(gamepad.getRawButton(GAMEPAD_X) || gamepad.getRawButton(GAMEPAD_B) || isGoingCargoShip); //if you are pressing X or B then it's not manual

		elevatorSubsystem.setIsManual(isManual);
		if (!isManual && isPreviousManual) {
			elevatorSubsystem.setSetpoint(elevatorSubsystem.getElevatorPosition());
			stayingAutomatic = false;
		}
		if ((-gamepad.getRawAxis(1) > TOPLINE && previousJoystick1Value <= TOPLINE) ||
			(-gamepad.getRawAxis(1) < BOTTOMLINE && previousJoystick1Value >= BOTTOMLINE)) { //if you flicked either way
			double[] setpoints = gamepad.getRawButton(GAMEPAD_X) ?
				elevatorSubsystem.getHatchSetpoints() : elevatorSubsystem.getCargoSetpoints();
			if(!stayingAutomatic) {
				double preset = elevatorSubsystem.getElevatorPreset(setpoints, -gamepad.getRawAxis(1) > 0);
				if(preset != -1) {
					elevatorSubsystem.setSetpoint(preset);
				}
				stayingAutomatic = true;
			} else {
				elevatorSubsystem.nextElevatorPreset(setpoints, -gamepad.getRawAxis(1) > 0); //lets you switch presets quickly
			}
		}
		double elevatorSpeed = deadband(-gamepad.getRawAxis(1), 0.05) >= 0 ?
			deadband(-gamepad.getRawAxis(1), 0.05) * 0.8 : deadband(-gamepad.getRawAxis(1), 0.05) * 0.4;
		if(isManual) {
			elevatorSubsystem.manualMove(elevatorSpeed);
		} else {
			elevatorSubsystem.setElevator();
		}
		if(elevatorSpeed > 0.05) {
			cargoIntakeSubsystem.spinRollerbarForElevator();
		}

		if(gamepad.getPOV() == POV_UP) { //if u press up
			isGoingCargoShip = true; //it's going to the cargo ship now!!!!
			elevatorSubsystem.setIsManual(false); //no longer manual
			elevatorSubsystem.CargoPlaceElevatorShip(); //set setpoint
			hatchIntakeSubsystem.setFrontIntakeOut(); //put out swan
		}
		if((Math.abs(gamepad.getRawAxis(1))) > 0.05 ) { //if u press the stick
			if(isGoingCargoShip == true) { //and it was going to the place
				elevatorSubsystem.setIsManual(true); //make it manual
				isGoingCargoShip = false;
			}
		}

		// Elevator zero
		if(gamepad.getPOV() == POV_DOWN) {
			elevatorSubsystem.zeroEncoder();
		}

		// Climb stuff
		if(rightJoystick.getRawButton(6)){
			climbingSubsystem.climbMoveForward();
		} else if(rightJoystick.getRawButton(7)) {
			climbingSubsystem.climbMoveBack();
		} else if(rightJoystick.getRawButton(11)) {
			climbingSubsystem.climbMoveUp();
		} else if(rightJoystick.getRawButton(10)) {
			climbingSubsystem.climbMoveDown();
		} else {
			climbingSubsystem.climbStop();
		}

		// Track some previous values
		previousJoystick1Value = -gamepad.getRawAxis(1);
		previousJoystick3Value = -gamepad.getRawAxis(3);
		isPreviousManual = isManual;

		// Subsystem-specific teleop periodics
		hatchIntakeSubsystem.teleopPeriodic();
		elevatorSubsystem.teleopPeriodic();
		drivetrainSubsystem.teleopPeriodic();
	}

	// Custom Functions

	// TODO(low): Should we remove this executeCommand function?

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
