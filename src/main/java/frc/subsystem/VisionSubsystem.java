package frc.subsystem;

import java.util.Arrays;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.Bezier;
import frc.ServiceLocator;
import frc.Vector;

public class VisionSubsystem {
	private final NetworkTable limelightTable = NetworkTableInstance.getDefault().getTable("limelight");
	public static final double LIMELIGHT_HEIGHT_FROM_GROUND = 10.6;
	public static final double HATCH_GOAL_HEIGHT_FROM_GROUND = 31.5;
	private final static double CARGO_GOAL_HEIGHT_FROM_GROUND = 38.75;
	public static final double CAMERA_HORIZONTAL_POS = 9.9517;
	public static final double CAMERA_VERTICAL_POS = 0;
	public static final double CAMERA_ROTATION_Z = 10.5;
	public static final double CAMERA_ROTATION_Y = 18.4182;
	public static final Vector CAMERA_ROBOT_SPACE = new Vector(CAMERA_HORIZONTAL_POS, CAMERA_VERTICAL_POS);
	public static final double RESOLUTION_X = 960;
	public static final double RESOLUTION_Y = 720;
	public static final double FOV_X = 59.6;
	public static final double FOV_Y = 49.7;
	public static final double PATH_STRENGTH = 12.0;
	private Bezier bezier;

	public VisionSubsystem() {
		ServiceLocator.register(this);
	}

	public double getHorizontalAngleOffset() {
		return limelightTable.getEntry("tx").getDouble(0.0);
	}

	public double getVerticalAngleOffset() {
		return limelightTable.getEntry("ty").getDouble(0.0);
	}

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

	/**
	 * @return an array of the corner points
	 */
	public Vector[] getCorners() {
		double[] defaultCornerArray = { 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 };
		double[] cornerXCoordinates = limelightTable.getEntry("tcornx").getDoubleArray(defaultCornerArray);
		double[] cornerYCoordinates = limelightTable.getEntry("tcorny").getDoubleArray(defaultCornerArray);
		Vector[] corners = new Vector[cornerXCoordinates.length];
		if(cornerXCoordinates.length == cornerYCoordinates.length) {
			for (int i = 0; i < cornerXCoordinates.length; i++){
				corners[i] = new Vector(cornerXCoordinates[i], cornerYCoordinates[i]);
			}
		} else {
			for(int i = 0; i < corners.length; i++) {
				corners[i] = new Vector(0.0, 0.0);
			}
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

	/**
	 * @param normalizedCameraPosition the normalized coordinates of the pixel you want to find the
	 * 	position of
	 * @return the position of the pixel relative to the camera in a top-down view
	 */
	public Vector getPositionOfPointCargo(Vector normalizedCameraPosition) {
		Vector viewPort = new Vector(2 * Math.tan(Math.toRadians(FOV_X) / 2),
		2 * Math.tan(Math.toRadians(FOV_Y) / 2));
		Vector pointInViewportSpace = new Vector((viewPort.x / 2) * normalizedCameraPosition.x,
		(viewPort.y / 2) * normalizedCameraPosition.y);
		double verticalOffset = Math.toDegrees(Math.atan(pointInViewportSpace.y));
		double horizontalOffset = Math.toDegrees(Math.atan(pointInViewportSpace.x));
		double distance = (CARGO_GOAL_HEIGHT_FROM_GROUND - LIMELIGHT_HEIGHT_FROM_GROUND) /
		Math.tan(Math.toRadians(verticalOffset + CAMERA_ROTATION_Y));
		double xCoord = Math.sin(Math.toRadians(horizontalOffset)) * distance;
		double yCoord = Math.cos(Math.toRadians(horizontalOffset)) * distance;
		SmartDashboard.putNumber("AutoPopulate/VerticalOffset", verticalOffset);
		SmartDashboard.putNumber("AutoPopulate/HorizontalOffset", horizontalOffset);
		SmartDashboard.putNumber("AutoPopulate/PositionX", xCoord);
		SmartDashboard.putNumber("AutoPopulate/PositionY", yCoord);
		return new Vector(xCoord, yCoord);
	}

	/**
	 * @param pixelX the image x coordinate of the pixel
	 * @param pixelY the image y coordinate of the pixel
	 * @return the position of the pixel relative to the camera in a top-down view
	 */
	public Vector getPositionOfPointCargo(double pixelX, double pixelY) {
		return getPositionOfPointCargo(convertFromPixelsToNormalized(new Vector(pixelX, pixelY)));
	}

	/**
	 * @param normalizedCameraPosition the normalized coordinates of the pixel you want to find the
	 * 	position of
	 * @return the position of the pixel relative to the camera in a top-down view
	 */
	public Vector getPositionOfPointHatch(Vector normalizedCameraPosition) {
		Vector viewPort = new Vector(2 * Math.tan(Math.toRadians(FOV_X) / 2),
		2 * Math.tan(Math.toRadians(FOV_Y) / 2));
		Vector pointInViewportSpace = new Vector((viewPort.x / 2) * normalizedCameraPosition.x,
		(viewPort.y / 2) * normalizedCameraPosition.y);
		double verticalOffset = Math.toDegrees(Math.atan(pointInViewportSpace.y));
		double horizontalOffset = Math.toDegrees(Math.atan(pointInViewportSpace.x));
		double distance = (HATCH_GOAL_HEIGHT_FROM_GROUND - LIMELIGHT_HEIGHT_FROM_GROUND) /
		Math.tan(Math.toRadians(verticalOffset + CAMERA_ROTATION_Y));
		double xCoord = Math.sin(Math.toRadians(horizontalOffset)) * distance;
		double yCoord = Math.cos(Math.toRadians(horizontalOffset)) * distance;
		SmartDashboard.putNumber("AutoPopulate/VerticalOffset", verticalOffset);
		SmartDashboard.putNumber("AutoPopulate/HorizontalOffset", horizontalOffset);
		SmartDashboard.putNumber("AutoPopulate/PositionX", xCoord);
		SmartDashboard.putNumber("AutoPopulate/PositionY", yCoord);
		return new Vector(xCoord, yCoord);
	}

	/**
	 * @param pixelX the image x coordinate of the pixel
	 * @param pixelY the image y coordinate of the pixel
	 * @return the position of the pixel relative to the camera in a top-down view
	 */
	public Vector getPositionOfPointHatch(double pixelX, double pixelY) {
		return getPositionOfPointHatch(convertFromPixelsToNormalized(new Vector(pixelX, pixelY)));
	}

	/**
	 * @param normalizedCameraPosition the normalized coordinates of the pixel you want to find the
	 * 	position of
	 * @return the position of the pixel relative to the robot in a top-down view
	 */
	public Vector getPositionOfPointRobotSpaceCargo(Vector normalizedCameraPosition) {
		Vector positionCameraSpace = getPositionOfPointCargo(normalizedCameraPosition);
		Vector positionRobotSpace = positionCameraSpace.rotate(CAMERA_ROTATION_Z);
		SmartDashboard.putNumber("AutoPopulate/TargetposX", positionRobotSpace.x);
		return positionRobotSpace = positionRobotSpace.add(CAMERA_ROBOT_SPACE);
	}

	/**
	 * @param pixelX the image x coordinate of the pixel
	 * @param pixelY the image y coordinate of the pixel
	 * @return the position of the pixel relative to the robot in a top-down view
	 */
	public Vector getPositionOfPointRobotSpaceCargo(double pixelX, double pixelY) {
		return getPositionOfPointRobotSpaceCargo(convertFromPixelsToNormalized(new Vector(pixelX, pixelY)));
	}

	/**
	 * @param normalizedCameraPosition the normalized coordinates of the pixel you want to find the
	 * 	position of
	 * @return the position of the pixel relative to the robot in a top-down view
	 */
	public Vector getPositionOfPointRobotSpaceHatch(Vector normalizedCameraPosition) {
		Vector positionCameraSpace = getPositionOfPointHatch(normalizedCameraPosition);
		Vector positionRobotSpace = positionCameraSpace.rotate(CAMERA_ROTATION_Z);
		SmartDashboard.putNumber("AutoPopulate/TargetposX", positionRobotSpace.x);
		return positionRobotSpace = positionRobotSpace.add(CAMERA_ROBOT_SPACE);
	}

	/**
	 * @param pixelX the image x coordinate of the pixel
	 * @param pixelY the image y coordinate of the pixel
	 * @return the position of the pixel relative to the robot in a top-down view
	 */
	public Vector getPositionOfPointRobotSpaceHatch(double pixelX, double pixelY) {
		return getPositionOfPointRobotSpaceHatch(convertFromPixelsToNormalized(new Vector(pixelX, pixelY)));
	}

	/**
	 * Converts image pixel coordinates into normalized coordinates
	 * @param rawPixelValues the point that the pixel occupies on the image
	 * @return the point in normalized coordinates
	 */
	public Vector convertFromPixelsToNormalized(Vector rawPixelValues) {
		Vector normalizedCoords = rawPixelValues;
		normalizedCoords.x = (1.0 / (0.5 * RESOLUTION_X)) * (normalizedCoords.x - (0.5 * RESOLUTION_X - 0.5));
		normalizedCoords.y = (1.0 / (0.5 * RESOLUTION_Y)) * ((0.5 * RESOLUTION_Y - 0.5) - normalizedCoords.y);
		SmartDashboard.putNumber("AutoPopulate/Vision/NormalizedX", normalizedCoords.x);
		SmartDashboard.putNumber("AutoPopulate/Vision/NormalizedY", normalizedCoords.y);
		return normalizedCoords;
    }

    public Vector getPositionOfCrosshairANormalized() {
        return new Vector(limelightTable.getEntry("cx0").getDouble(0), limelightTable.getEntry("cy0").getDouble(0));
	}

	/**
	 * Gets the position in a top down view of the two outtermost, topmost corners of the
	 * vision target
	 * @return an array of vectors { leftCorner, rightCorner } (only every two elements)
	 */
	public Vector[] getPositionOfCornersCargo() {
		Vector[] corners = getCorners();
		if(corners.length >= 8) {
			Vector leftTarget;
			Vector rightTarget;
			Arrays.sort(corners, (Vector a, Vector b) -> {
				if(a.x > b.x) {
					return 1;
				} else if(a.x == b.x) {
					return 0;
				} else {
					return -1;
				}
			});
			leftTarget = corners[1].y > corners[2].y ? corners[1] : corners[2];
			rightTarget = corners[5].y > corners[6].y ? corners[5] : corners[6];
			Vector leftTargetPosition = getPositionOfPointRobotSpaceCargo(leftTarget.x, leftTarget.y);
			Vector rightTargetPosition = getPositionOfPointRobotSpaceCargo(rightTarget.x, rightTarget.y);
			SmartDashboard.putNumber("AutoPopulate/Vision/LeftCornerX", leftTargetPosition.x);
			SmartDashboard.putNumber("AutoPopulate/Vision/LeftCornerY", leftTargetPosition.y);
			SmartDashboard.putNumber("AutoPopulate/Vision/RightCornerX", rightTargetPosition.x);
			SmartDashboard.putNumber("AutoPopulate/Vision/RightCornerY", rightTargetPosition.y);
			SmartDashboard.putNumber("AutoPopulate/Vision/LeftCornerXPixels", leftTarget.x);
			SmartDashboard.putNumber("AutoPopulate/Vision/LeftCornerYPixels", leftTarget.y);
			SmartDashboard.putNumber("AutoPopulate/Vision/RightCornerXPixels", rightTarget.x);
			SmartDashboard.putNumber("AutoPopulate/Vision/RightCornerYPixels", rightTarget.y);
			Vector[] cornerPositions = { leftTargetPosition, rightTargetPosition };
			return cornerPositions;
		} else {
			Vector[] defaultReturn = { new Vector(0, 0), new Vector(0, 0) };
			return defaultReturn;
		}
	}

	/**
	 * Gets the position in a top down view of the two outtermost, topmost corners of the
	 * vision target
	 * @return an array of vectors { leftCorner, rightCorner } (only every two elements)
	 */
	public Vector[] getPositionOfCornersHatch() {
		Vector[] corners = getCorners();
		if(corners.length >= 8) {
			Vector leftTarget;
			Vector rightTarget;
			Arrays.sort(corners, (Vector a, Vector b) -> {
				if(a.x > b.x) {
					return 1;
				} else if(a.x == b.x) {
					return 0;
				} else {
					return -1;
				}
			});
			leftTarget = corners[1].y > corners[2].y ? corners[1] : corners[2];
			rightTarget = corners[5].y > corners[6].y ? corners[5] : corners[6];
			Vector leftTargetPosition = getPositionOfPointRobotSpaceHatch(leftTarget.x, leftTarget.y);
			Vector rightTargetPosition = getPositionOfPointRobotSpaceHatch(rightTarget.x, rightTarget.y);
			SmartDashboard.putNumber("AutoPopulate/Vision/LeftCornerX", leftTargetPosition.x);
			SmartDashboard.putNumber("AutoPopulate/Vision/LeftCornerY", leftTargetPosition.y);
			SmartDashboard.putNumber("AutoPopulate/Vision/RightCornerX", rightTargetPosition.x);
			SmartDashboard.putNumber("AutoPopulate/Vision/RightCornerY", rightTargetPosition.y);
			SmartDashboard.putNumber("AutoPopulate/Vision/LeftCornerXPixels", leftTarget.x);
			SmartDashboard.putNumber("AutoPopulate/Vision/LeftCornerYPixels", leftTarget.y);
			SmartDashboard.putNumber("AutoPopulate/Vision/RightCornerXPixels", rightTarget.x);
			SmartDashboard.putNumber("AutoPopulate/Vision/RightCornerYPixels", rightTarget.y);
			Vector[] cornerPositions = { leftTargetPosition, rightTargetPosition };
			return cornerPositions;
		} else {
			Vector[] defaultReturn = { new Vector(0, 0), new Vector(0, 0) };
			return defaultReturn;
		}
	}

	public double getTargetZRotationCargo() {
		Vector[] corners = getPositionOfCornersCargo();
		Vector leftCornerPosition = corners[0];
		Vector rightCornerPosition = corners[1];
		double deltaX = rightCornerPosition.x - leftCornerPosition.x;
		double deltaY = rightCornerPosition.y - leftCornerPosition.y;
		if(deltaY != 0) {
			return Math.toDegrees(Math.atan(deltaY / deltaX));
		} else {
			return 0;
		}
	}

	public double getTargetZRotationHatch() {
		Vector[] corners = getPositionOfCornersHatch();
		Vector leftCornerPosition = corners[0];
		Vector rightCornerPosition = corners[1];
		double deltaX = rightCornerPosition.x - leftCornerPosition.x;
		double deltaY = rightCornerPosition.y - leftCornerPosition.y;
		if(deltaY != 0) {
			return Math.toDegrees(Math.atan(deltaY / deltaX));
		} else {
			return 0;
		}
	}

	public Vector[] genPathToTargetCargo(int resolution) {
		Vector targetLocation = getTargetPositionRobotSpaceCargo();
		double targetZRotation = getTargetZRotationCargo();
		bezier = new Bezier(targetLocation.x, targetLocation.y - 2, PATH_STRENGTH, 2 * PATH_STRENGTH, targetZRotation);
		return bezier.generateBezier(resolution);
	}

	public Vector[] genPathToTargetHatch(int resolution) {
		Vector targetLocation = getTargetPositionRobotSpaceHatch();
		double targetZRotation = getTargetZRotationHatch();
		SmartDashboard.putNumber("VisionOtherStuff/TargetPositionX", targetLocation.x);
		SmartDashboard.putNumber("VisionOtherStuff/TargetPositionY", targetLocation.y);
		SmartDashboard.putNumber("VisionOtherStuff/TargetRotation", targetZRotation);
		bezier = new Bezier(targetLocation.x, targetLocation.y - 2, PATH_STRENGTH, 2 * PATH_STRENGTH, targetZRotation);
		return bezier.generateBezier(resolution);
	}
}
