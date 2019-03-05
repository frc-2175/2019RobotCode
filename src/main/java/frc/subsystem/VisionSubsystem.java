package frc.subsystem;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.ServiceLocator;
import frc.Vector;

public class VisionSubsystem {
	private final NetworkTable limelightTable = NetworkTableInstance.getDefault().getTable("limelight");
	public static final double LIMELIGHT_HEIGHT_FROM_GROUND = 10.375;
	public static final double HATCH_GOAL_HEIGHT_FROM_GROUND = 31.5;
	private final static double CARGO_GOAL_HEIGHT_FROM_GROUND = 38.75;
	public static final double CAMERA_HORIZONTAL_POS = 9.9517;
	public static final double CAMERA_VERTICAL_POS = 13.2;
	public static final double CAMERA_ROTATION_Z = 12;
	public static final double CAMERA_ROTATION_Y = 18.4182;
	public static final Vector CAMERA_ROBOT_SPACE = new Vector(CAMERA_HORIZONTAL_POS, CAMERA_VERTICAL_POS);

	public VisionSubsystem() {
		ServiceLocator.register(this);
	}

	/**
	 * @return horizontal angle offset
	 */
	public double getHorizontalAngleOffset() {
		return limelightTable.getEntry("tx").getDouble(0.0);
	}

	/**
	 * @return vertical angle offset
	 */
	public double getVerticalAngleOffset() {
		return limelightTable.getEntry("ty").getDouble(0.0);
	}

	/**
	 * @return whether limelight has any valid targets
	 */
	public boolean doesValidTargetExist() {
		return limelightTable.getEntry("tv").getDouble(0.0) >= 1;
	}

	/**
	 * @param pipelineNumber pipeline index of camera you want (0-9)
	 */
	public void setPipeline(int pipelineNumber) {
		limelightTable.getEntry("pipeline").setNumber(pipelineNumber);
	}

	/**
	 * @param camModeNumber number for camera mode you want (0 vision processor, 1 driver cam)
	 */
	public void setCamMode(int camModeNumber) {
		limelightTable.getEntry("camMode").setNumber(camModeNumber);
	}

	/**
	 * @param ledModeNumber number for LED mode (0 use current LED mode, 1 force off, 2 force blink, 3 force on)
	 */
	public void setLedMode(int ledModeNumber) {
		limelightTable.getEntry("ledMode").setNumber(ledModeNumber);
	}

	public Vector[] getCorners() {
		double[] defaultCornerArray = { 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 };
		double[] cornerXCoordinates = limelightTable.getEntry("tcornx").getDoubleArray(defaultCornerArray);
		double[] cornerYCoordinates = limelightTable.getEntry("tcorny").getDoubleArray(defaultCornerArray);
		Vector[] corners = new Vector[cornerXCoordinates.length];
		for (int i = 0; i < cornerXCoordinates.length; i++){
			corners[i] = new Vector(cornerXCoordinates[i], cornerYCoordinates[i]);
		}
		return corners;
	}

	public Vector getTargetPositionRobotSpaceCargo() {
		double horizontalOffset = getHorizontalAngleOffset();
		double verticalOffset = getVerticalAngleOffset();
		double distance = (CARGO_GOAL_HEIGHT_FROM_GROUND - LIMELIGHT_HEIGHT_FROM_GROUND) /
			Math.tan(Math.toRadians(verticalOffset + CAMERA_ROTATION_Y));
		double xCoord = Math.sin(Math.toRadians(horizontalOffset)) * distance;
		double yCoord = Math.cos(Math.toRadians(horizontalOffset)) * distance;
		Vector positionCameraSpace = new Vector(xCoord, yCoord);
		Vector positionRobotSpace = positionCameraSpace.rotate(CAMERA_ROTATION_Z);
		SmartDashboard.putNumber("AutoPopulate/TargetposX", positionRobotSpace.x);
		return positionRobotSpace = positionRobotSpace.add(CAMERA_ROBOT_SPACE);
	}

	public Vector getTargetPositionRobotSpaceHatch() {
		double horizontalOffset = getHorizontalAngleOffset();
		double verticalOffset = getVerticalAngleOffset();
		double distance = (HATCH_GOAL_HEIGHT_FROM_GROUND - LIMELIGHT_HEIGHT_FROM_GROUND) /
			Math.tan(Math.toRadians(verticalOffset + CAMERA_ROTATION_Y));
		double xCoord = Math.sin(Math.toRadians(horizontalOffset)) * distance;
		double yCoord = Math.cos(Math.toRadians(horizontalOffset)) * distance;
		Vector positionCameraSpace = new Vector(xCoord, yCoord);
		Vector positionRobotSpace = positionCameraSpace.rotate(CAMERA_ROTATION_Z);
		SmartDashboard.putNumber("AutoPopulate/TargetposX", positionRobotSpace.x);
		return positionRobotSpace = positionRobotSpace.add(CAMERA_ROBOT_SPACE);
	}

	public double getAngleToTargetZCargo() {
		Vector targetPos = getTargetPositionRobotSpaceCargo();
		return Math.toDegrees(Math.atan(targetPos.x / targetPos.y));
	}

	public double getAngleToTargetZHatch() {
		Vector targetPos = getTargetPositionRobotSpaceHatch();
		return Math.toDegrees(Math.atan(targetPos.x / targetPos.y));
	}
}
