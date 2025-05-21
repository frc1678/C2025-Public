package frc.robot.subsystems.detection;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Distance;
import frc.lib.bases.LimelightSubsystem.LimelightConfig;
import frc.lib.io.DetectionIOLimelight;
import frc.lib.io.DetectionIOLimelightSim;
import frc.robot.Robot;

public class DetectionConstants {
	public static final String kLimelightName = "limelight-coral";
	public static final Pose3d kRobotToCameraOffset = new Pose3d(
			// forward/back, left/right, up/down
			new Translation3d(Units.Inches.of(-0.109), Units.Inches.of(-4.550), Units.Inches.of(39.230)),
			// roll (along robot y axis), pitch (along robot x axis), yaw (along robot z axis)
			new Rotation3d(Units.Degree.of(0.0), Units.Degree.of(-33.0), Units.Degree.of(0.0)));
	public static final int kTelePipeline = 3;
	public static final int kAutoPipeline = 0;
	public static final int kDisabledPipeline = 1;
	public static final Distance kCoralRadius = Units.Inches.of(4.5 / 2);

	public static final LimelightConfig getDetectionIOConfig() {
		LimelightConfig config = new LimelightConfig();
		config.name = kLimelightName;
		config.robotToCameraOffset = kRobotToCameraOffset;
		config.aprilTagVisionStdDevs = VecBuilder.fill(0.8, 0.8, 99999.0);
		return config;
	}

	public static final DetectionIOLimelight getDetectionIO() {
		if (Robot.isReal()) {
			return new DetectionIOLimelight();
		} else {
			return new DetectionIOLimelightSim();
		}
	}
}
