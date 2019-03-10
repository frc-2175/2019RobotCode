package frc.subsystem;

import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.drive.RobotDriveBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.MotorWrapper;
import frc.PIDController;
import frc.ServiceLocator;
import frc.VirtualSpeedController;
import frc.info.RobotInfo;
import frc.info.SmartDashboardInfo;
import frc.Vector;

public class DrivetrainSubsystem {

	private final RobotInfo robotInfo;
	private final MotorWrapper leftMaster;
	private final MotorWrapper leftSlave;
	private final MotorWrapper rightMaster;
	private final MotorWrapper rightSlave;
	private final DifferentialDrive robotDrive;
	private static VirtualSpeedController leftVirtualSpeedController = new VirtualSpeedController();
	private static VirtualSpeedController rightVirtualSpeedController = new VirtualSpeedController();
	private static DifferentialDrive virtualRobotDrive = new DifferentialDrive(leftVirtualSpeedController,
		rightVirtualSpeedController);
	private PIDController pidController;
	private PIDController purePursuitPID;
	private PIDController endTerm;
	private VisionSubsystem visionSubsystem;
	public double targetHeading;
	public AHRS navx;
	public Vector fieldPosition;
	public static final double TICKS_TO_INCHES = 0.0045933578;
	double lastEncoderDistanceLeft;
	double lastEncoderDistanceRight;
	private double zeroEncoderLeft;
	private double zeroEncoderRight;
	public static final double INPUT_THRESHOLD = 0.1;
	private double targetZRotation = 0;

	public DrivetrainSubsystem() {
		ServiceLocator.register(this);

		robotInfo = ServiceLocator.get(RobotInfo.class);
		leftMaster = robotInfo.get(RobotInfo.LEFT_MOTOR_MASTER);
		leftSlave = robotInfo.get(RobotInfo.LEFT_MOTOR_FOLLOWER);
		rightMaster = robotInfo.get(RobotInfo.RIGHT_MOTOR_MASTER);
		rightSlave = robotInfo.get(RobotInfo.RIGHT_MOTOR_FOLLOWER);

		leftMaster.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, 0, 0);
		rightMaster.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, 0, 0);
		leftMaster.setSelectedSensorPosition(0, 0, 0);
		rightMaster.setSelectedSensorPosition(0, 0, 0);

		leftSlave.follow(leftMaster);
		rightSlave.follow(rightMaster);

		robotDrive = new DifferentialDrive(leftMaster.getMotor(), rightMaster.getMotor());

		leftVirtualSpeedController = new VirtualSpeedController();
		rightVirtualSpeedController = new VirtualSpeedController();
		virtualRobotDrive = new DifferentialDrive(leftVirtualSpeedController, rightVirtualSpeedController);
		SmartDashboardInfo smartDashboardInfo = ServiceLocator.get(SmartDashboardInfo.class);
		double kp = smartDashboardInfo.getNumber(SmartDashboardInfo.VISION_PID_P);
		double ki = smartDashboardInfo.getNumber(SmartDashboardInfo.VISION_PID_I);
		double kd = smartDashboardInfo.getNumber(SmartDashboardInfo.VISION_PID_D);
		pidController = new PIDController(kp, ki, kd);
		SmartDashboard.putNumber("PurePursuit/KP", 0.015); // 0.025
		SmartDashboard.putNumber("PurePursuit/KI", 0);
		SmartDashboard.putNumber("PurePursuit/KD", 0);
		double p_KP = SmartDashboard.getNumber("PurePursuit/KP", 0);
		double p_KI = SmartDashboard.getNumber("PurePursuit/KI", 0);
		double p_KD = SmartDashboard.getNumber("PurePursuit/KD", 0);
		purePursuitPID = new PIDController(p_KP, p_KI, p_KD);
		navx = new AHRS(SPI.Port.kMXP);
		navx.reset();

		fieldPosition = new Vector(0, 0);

		visionSubsystem = ServiceLocator.get(VisionSubsystem.class);
		targetHeading = 0;

		lastEncoderDistanceLeft = 0;
    	lastEncoderDistanceRight = 0;
		zeroEncoderLeft = 0;
		zeroEncoderRight = 0;

		SmartDashboard.putNumber("PurePursuit/MinSpeed", 0.4);
		SmartDashboard.putNumber("PurePursuit/MaxSpeed", 0.6);
		SmartDashboard.putNumber("PurePursuit/LookAhead", 12.0);
		SmartDashboard.putNumber("PurePursuit/TransitionLength", 0.4);

		// leftMaster.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, 0, 0);
		// rightMaster.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, 0, 0);
		// leftMaster.setSelectedSensorPosition(0, 0, 0);
		// rightMaster.setSelectedSensorPosition(0, 0, 0);

		// CameraServer.getInstance().startAutomaticCapture();

		// endTerm = new PIDController(-0.018, -0.014, 0.005);
		endTerm = new PIDController(-0.01, 0, 0);
	}

	public void stopAllMotors() {
	}

	/**
	 * Returns the lerp of the arcade and curvature values for the left and right
	 * virtual speed controllers
	 *
	 * @return {left, right}
	 */
	public static double[] getBlendedMotorValues(double moveValue, double turnValue, double inputThreshold) {
		virtualRobotDrive.arcadeDrive(moveValue, turnValue, false);
		double leftArcadeValue = leftVirtualSpeedController.get() * 0.8;
		double rightArcadeValue = rightVirtualSpeedController.get()* 0.8;

		virtualRobotDrive.curvatureDrive(moveValue, turnValue, false);
		double leftCurvatureValue = leftVirtualSpeedController.get();
		double rightCurvatureValue = rightVirtualSpeedController.get();

		double lerpT = Math.abs(deadband(moveValue, RobotDriveBase.kDefaultDeadband)) / inputThreshold;
		lerpT = clamp(lerpT, 0, 1);
		double leftBlend = lerp(leftArcadeValue, leftCurvatureValue, lerpT);
		double rightBlend = lerp(rightArcadeValue, rightCurvatureValue, lerpT);

		double[] blends = { leftBlend, -rightBlend };
		return blends;
	}

	/**
	 * Drives with a blend between curvature and arcade drive using
	 * linear interpolation
	 *
	 * @param xSpeed the forward/backward speed for the robot
	 * @param zRotation the curvature to drive/the in-place rotation
	 * @see #getBlendedMotorValues(double, double)
	 */
	public void blendedDrive(double xSpeed, double zRotation, double inputThreshold) {
		double[] blendedValues = getBlendedMotorValues(xSpeed, zRotation, inputThreshold);
		robotDrive.tankDrive(blendedValues[0], blendedValues[1]);
	}

	public void blendedDrive(double xSpeed, double zRotation) {
		blendedDrive(xSpeed, zRotation, INPUT_THRESHOLD);
	}

	/**
	 * Clamps a double value based on a minimum and a maximum
	 *
	 * @param val the value to clamp
	 * @param min the minimum to clamp on
	 * @param max the maximum to clamp on
	 * @return min if val is less than min or max if val is greater than max
	 */
	public static double clamp(double val, double min, double max) {
		return val >= min && val <= max ? val : (val < min ? min : max);
	}

	/**
	 * Linearly interpolates between two points based on a t value
	 *
	 * @param a the point to interpolate from
	 * @param b the point to interpolate to
	 * @param t the value to interpolate on
	 * @return an output based on the formula lerp(a, b, t) = (1-t)a + tb
	 */
	public static double lerp(double a, double b, double t) {
		return (1 - t) * a + t * b;
	}

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

	public static double undeadband(double value, double deadband) {
		if (value < 0) {
			double t = -value;
			return DrivetrainSubsystem.lerp(-deadband, -1, t);
		} else if (value > 0) {
			double t = value;
			return DrivetrainSubsystem.lerp(deadband, 1, t);
		} else {
			return 0;
		}
	}

	public void arcadeDrive(double moveValue, double turnValue) {
		robotDrive.arcadeDrive(-moveValue, -turnValue);
	}

	public void tankDrive(double leftSpeed, double rightSpeed) {
		robotDrive.tankDrive(leftSpeed, rightSpeed);
	}

	/**
	 * Stores the gyro heading of the cargo vision target for use in vision steering
	 *
	 * @see #driveWithSimpleVision(double)
	 */
	public void storeTargetHeadingCargo() {
		double offsetAngleVision = visionSubsystem.getAngleZToTarget(VisionSubsystem.CARGO_GOAL_HEIGHT_FROM_GROUND);
		SmartDashboard.putNumber("AutoPopulate/OffsetAngle", offsetAngleVision);
		targetHeading = navx.getAngle() + offsetAngleVision;
		SmartDashboard.putNumber("AutoPopulate/TargetHeading", targetHeading);
	}

	/**
	 * Stores the gyro heading of the hatch vision target for use in vision steering
	 *
	 * @see #driveWithSimpleVision(double)
	 */
	public void storeTargetHeadingHatch() {
		double offsetAngleVision = visionSubsystem.getAngleZToTarget(VisionSubsystem.HATCH_GOAL_HEIGHT_FROM_GROUND);
		SmartDashboard.putNumber("AutoPopulate/OffsetAngle", offsetAngleVision);
		targetHeading = navx.getAngle() + offsetAngleVision;
		SmartDashboard.putNumber("AutoPopulate/TargetHeading", targetHeading);
	}

	/**
	 * Steers with a pid loop towards a vision target using blended drive
	 *
	 * @param xSpeed the forward/backward speed that the robot should drive at
	 * (usually a joystick input)
	 * @see #blendedDrive(double, double)
	 */
	public void driveWithSimpleVision(double xSpeed) {
		double zRotation = -pidController.pid(navx.getAngle(), targetHeading);
		SmartDashboard.putNumber("AutoPopulate/PIDOutput", zRotation);
		SmartDashboard.putNumber("AutoPopulate/AngleOffset", navx.getAngle() - targetHeading);
		blendedDrive(xSpeed, -zRotation);
	}

	public void trackLocation() {
		double distanceLeft = getLeftSideDistanceDriven() - lastEncoderDistanceLeft;
		double distanceRight = getRightSideDistanceDriven() - lastEncoderDistanceRight;
		double distance = (distanceLeft + distanceRight) / 2.0;
		double angle = Math.toRadians(navx.getAngle());

		double x = Math.sin(angle) * distance;
		double y = Math.cos(angle) * distance;

		Vector changeInPosition = new Vector(x, y);
		fieldPosition = fieldPosition.add(changeInPosition);

		lastEncoderDistanceLeft = getLeftSideDistanceDriven();
		lastEncoderDistanceRight = getRightSideDistanceDriven();
	}

	public void purePursuit(Vector[] path, Vector targetLocation, double moveValue) {
		double percentOfPathTravelled = findClosestPoint(path, fieldPosition) / (path.length - 1.0);
		if(percentOfPathTravelled < 1) {
			// Get goal point
			int indexOfGoalPoint = findGoalPoint(path, fieldPosition, SmartDashboard.getNumber("PurePursuit/LookAhead", 12.0));
			SmartDashboard.putNumber("PurePursuitLogging/IndexOfGoalPoint", indexOfGoalPoint);
			SmartDashboard.putNumber("PurePursuitLogging/GoalPointX", path[indexOfGoalPoint].x);
			SmartDashboard.putNumber("PurePursuitLogging/GoalPointY", path[indexOfGoalPoint].y);
			// Transform goal point into robot coordinates
			Vector goalPoint = path[indexOfGoalPoint].subtract(fieldPosition).rotate(navx.getAngle());
			// Set steering
			double offsetAngle = Math.toDegrees(Math.atan(Math.abs(goalPoint.x / goalPoint.y)));
			if(goalPoint.x >= 0 && goalPoint.y < 0) {
				offsetAngle += 90;
			} else if(goalPoint.x < 0) {
				if(goalPoint.y >= 0) {
					offsetAngle *= -1;
				} else {
					offsetAngle *= -1;
					offsetAngle -= 90;
				}
			}
			SmartDashboard.putNumber("PurePursuitLogging/OffsetAngle", offsetAngle);
			double zRotation = purePursuitPID.pid(-offsetAngle, 0);
			SmartDashboard.putNumber("PurePursuitLogging/ZRotation", zRotation);

			SmartDashboard.putNumber("PurePursuitLogging/PercentTravelled", percentOfPathTravelled);
			double speed = trapezoidAcceleration(percentOfPathTravelled, SmartDashboard.getNumber("PurePursuit/MaxSpeed", 0.9),
				SmartDashboard.getNumber("PurePursuit/MinSpeed", 0.3), SmartDashboard.getNumber("PurePursuit/TransitionLength", 0.25));
			SmartDashboard.putNumber("PurePursuitLogging/Speed", speed);

			blendedDrive(speed, zRotation, SmartDashboard.getNumber("PurePursuit/MaxSpeed", 0.6));
			// Update location
			trackLocation();
			purePursuitPID.updateTime(Timer.getFPGATimestamp());
		} else {
			double dx = targetLocation.x - fieldPosition.x;
			double dy = targetLocation.y - fieldPosition.y;
			double targetRotation = Math.toDegrees(Math.atan(dx / dy));
			SmartDashboard.putNumber("PurePursuitLogging/TargetRotation", targetRotation);
			double zRotation;
			if(Math.abs(targetRotation - navx.getAngle()) < 1) {
				zRotation = 0;
			} else {
				zRotation = endTerm.pid(navx.getAngle(), targetRotation, 15); //targetZRotation
				SmartDashboard.putNumber("PurePursuitLogging/ZRotation", zRotation);
				double constantStallOvercome = 0.3 * -Math.signum(targetRotation - navx.getAngle());
				zRotation += constantStallOvercome;
			}
			arcadeDrive(moveValue, zRotation);
			endTerm.updateTime(Timer.getFPGATimestamp());
			trackLocation();
		}
	}

	public static double proportional(double input, double setpoint, double kp) {
		return (setpoint - input) * kp;
	}

	public static int findGoalPoint(Vector[] path, Vector fieldPosition, double lookAhead) {
		int indexOfGoalPoint = findClosestPoint(path, fieldPosition);
		for(; indexOfGoalPoint < path.length; indexOfGoalPoint++) {
			if(indexOfGoalPoint + 1 >= path.length || path[indexOfGoalPoint + 1].subtract(fieldPosition).getLength() >= lookAhead) {
				break;
			}
		}
		return indexOfGoalPoint;
	}

	public static int findClosestPoint(Vector[] path, Vector fieldPosition) {
		int indexOfClosestPoint = 0;
		for(int i = 0; i < path.length; i++) {
			Vector difference = path[i].subtract(fieldPosition);
			double distanceToPoint = difference.getLength();
			double currentLowestDistance = path[indexOfClosestPoint].subtract(fieldPosition).getLength();
			indexOfClosestPoint = distanceToPoint < currentLowestDistance ? i : indexOfClosestPoint;
		}
		return indexOfClosestPoint;
	}

	public static double trapezoidAcceleration(double percentTravelled, double setpoint, double minimumStartSpeed, double transitionLength) {
		if(percentTravelled >= 0 && percentTravelled < transitionLength) {
			return lerp(minimumStartSpeed, setpoint, percentTravelled / transitionLength);
		} else if(percentTravelled >= transitionLength && percentTravelled < 1 - transitionLength) {
			return setpoint;
		} else if(percentTravelled >= 1 - transitionLength && percentTravelled < 1) {
			return lerp(setpoint, 0, (percentTravelled - (1 - transitionLength)) / transitionLength);
		} else {
			return 0;
		}
	}

	public double getLeftSideDistanceDriven() {
		return -(leftMaster.getSelectedSensorPosition(0) - zeroEncoderLeft) * TICKS_TO_INCHES;
	}

	public double getRightSideDistanceDriven() {
		return (rightMaster.getSelectedSensorPosition(0) - zeroEncoderRight) * TICKS_TO_INCHES;
	}

	public void resetTracking() {
		lastEncoderDistanceLeft = 0;
		lastEncoderDistanceRight = 0;
		zeroEncoderLeft = leftMaster.getSelectedSensorPosition(0);
		zeroEncoderRight = rightMaster.getSelectedSensorPosition(0);
		// leftMaster.setSelectedSensorPosition(0, 0, 0);
		// rightMaster.setSelectedSensorPosition(0, 0, 0);
		navx.reset();
		endTerm.clear(Timer.getFPGATimestamp());
		pidController.clear(Timer.getFPGATimestamp());
		purePursuitPID.clear(Timer.getFPGATimestamp());
		fieldPosition = new Vector(0, 0);
	}

	public void storeTargetZRotationCargo() {
		targetZRotation = -visionSubsystem.getTargetZRotation(VisionSubsystem.CARGO_GOAL_HEIGHT_FROM_GROUND);
	}

	public void storeTargetZRotationHatch() {
		targetZRotation = -visionSubsystem.getTargetZRotation(VisionSubsystem.HATCH_GOAL_HEIGHT_FROM_GROUND);
	}

	public void teleopPeriodic() {
		pidController.updateTime(Timer.getFPGATimestamp());
		purePursuitPID.updateTime(Timer.getFPGATimestamp());
		SmartDashboard.putNumber("AutoPopulate/Gyro", navx.getAngle());
		SmartDashboard.putNumber("Values/LeftRawEncoder", leftMaster.getSelectedSensorPosition(0));
		SmartDashboard.putNumber("Values/RightRawEncoder", rightMaster.getSelectedSensorPosition(0));
	}

	public void proportionalZeroTurn() {
		double output = proportional(navx.getAngle(), 0, 0.01);
		arcadeDrive(0, output);
	}
}
