package frc.subsystem;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import frc.ServiceLocator;
import frc.Vector;

public class VisionSubsystem {
	private final NetworkTable limelightTable = NetworkTableInstance.getDefault().getTable("limelight");
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
}
