package frc.lib.io;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructPublisher;
import frc.robot.subsystems.drive.Drive;

/**
 * Does nothing, used for simulation.
 */
public class DetectionIOLimelightSim extends DetectionIOLimelight {
	private final NetworkTableInstance ntInstance = NetworkTableInstance.getDefault();
	private final NetworkTable visTable = ntInstance.getTable("SmartDashboard/Detection");
	private final StructPublisher<Pose2d> closestCoralPose =
			visTable.getStructTopic("CoralPose", Pose2d.struct).publish();
	private final StructPublisher<Translation2d> closestCoralTranslation =
			visTable.getStructTopic("CoralTranslation", Translation2d.struct).publish();

	@Override
	public Pose2d getCoralPose(Translation2d base) {
		// Feed in the coral pose you want the robot to track to here
		Pose2d coralPose = new Pose2d(14.0, 7.23, Drive.mInstance.getPose().getRotation());
		closestCoralPose.set(coralPose);
		return coralPose;
	}

	@Override
	public int coralCount() {
		return 1;
	}
}
