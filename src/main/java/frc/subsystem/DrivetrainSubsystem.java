package frc.subsystem;

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
	private VisionSubsystem visionSubsystem;
	public double targetHeading;
	public AHRS navx;

	public DrivetrainSubsystem() {
		ServiceLocator.register(this);

		robotInfo = ServiceLocator.get(RobotInfo.class);
		leftMaster = robotInfo.get(RobotInfo.LEFT_MOTOR_MASTER);
		leftSlave = robotInfo.get(RobotInfo.LEFT_MOTOR_FOLLOWER);
		rightMaster = robotInfo.get(RobotInfo.RIGHT_MOTOR_MASTER);
		rightSlave = robotInfo.get(RobotInfo.RIGHT_MOTOR_FOLLOWER);
		leftMaster.setInverted(false);
		rightMaster.setInverted(false);

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
		navx = new AHRS(SPI.Port.kMXP);
		navx.reset();

		visionSubsystem = ServiceLocator.get(VisionSubsystem.class);
		targetHeading = 0;

		// leftMaster.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, 0, 0);
		// rightMaster.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, 0, 0);
		// leftMaster.setSelectedSensorPosition(0, 0, 0);
		// rightMaster.setSelectedSensorPosition(0, 0, 0);

		// CameraServer.getInstance().startAutomaticCapture();
	}

	public void stopAllMotors() {
		blendedDrive(0, 0);
	}

	/**
	 * Returns the lerp of the arcade and curvature values for the left and right
	 * virtual speed controllers
	 *
	 * @return {left, right}
	 */
	public static double[] getBlendedMotorValues(double moveValue, double turnValue) {
		final double INPUT_THRESHOLD = 0.1;
		virtualRobotDrive.arcadeDrive(moveValue, turnValue, false);
		double leftArcadeValue = leftVirtualSpeedController.get() * 0.8;
		double rightArcadeValue = rightVirtualSpeedController.get()* 0.8;

		virtualRobotDrive.curvatureDrive(moveValue, turnValue, false);
		double leftCurvatureValue = leftVirtualSpeedController.get();
		double rightCurvatureValue = rightVirtualSpeedController.get();

		double lerpT = Math.abs(deadband(moveValue, RobotDriveBase.kDefaultDeadband)) / INPUT_THRESHOLD;
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
	public void blendedDrive(double xSpeed, double zRotation) {
		double[] blendedValues = getBlendedMotorValues(xSpeed, zRotation);
		robotDrive.tankDrive(blendedValues[0], blendedValues[1]);
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
		double offsetAngleVision = visionSubsystem.getAngleToTargetZCargo();
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
		double offsetAngleVision = visionSubsystem.getAngleToTargetZHatch();
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

	public void teleopPeriodic() {
		pidController.updateTime(Timer.getFPGATimestamp());
		SmartDashboard.putNumber("AutoPopulate/Gyro", navx.getAngle());
	}
}
