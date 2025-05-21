package frc.robot;

import choreo.auto.AutoFactory;

public class RobotConstants {
	public static String kSerial;
	public static boolean isComp;
	public static boolean isOmega;

	static {
		if (Robot.isReal()) {
			kSerial = System.getenv("serialnum");
		} else {
			kSerial = "";
		}
		RobotConstants.isComp = kSerial.startsWith(RobotConstants.kCompSerial);
		RobotConstants.isOmega = kSerial.startsWith(RobotConstants.kOmegaSerial);
		RobotConstants.isRedAlliance = false;
	}

	public static boolean isRedAlliance;
	public static AutoFactory mAutoFactory;

	public static final String kCompSerial = "03415A0E";
	public static final String kOmegaSerial = "032B4B47";
}
