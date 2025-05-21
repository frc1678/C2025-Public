package frc.lib.io;

import com.ctre.phoenix6.Utils;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.units.BaseUnits;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.lib.bases.LimelightSubsystem.LimelightConfig;
import frc.lib.logging.LogUtil;
import frc.lib.util.FieldLayout;
import frc.lib.util.LimelightHelpers;
import frc.lib.util.LimelightHelpers.PoseEstimate;
import frc.lib.util.LimelightHelpers.RawDetection;
import frc.lib.util.Stopwatch;
import frc.lib.util.Util;
import frc.robot.subsystems.detection.DetectionConstants;
import frc.robot.subsystems.drive.Drive;
import java.util.ArrayList;

public class DetectionIOLimelight extends DetectionIO {
	private final NetworkTableInstance ntInstance = NetworkTableInstance.getDefault();
	private int maxI = 0;
	private ArrayList<StructPublisher<Pose2d>> publishers = new ArrayList<StructPublisher<Pose2d>>();
	private ArrayList<Coral> tracker = new ArrayList<Coral>();
	private Stopwatch mStopwatch = new Stopwatch();
	private int pipelineToSet = 0;
	private Stopwatch mResetStopwatch = new Stopwatch();
	private LimelightConfig config = new LimelightConfig();
	private final NetworkTable visTable = ntInstance.getTable("SmartDashboard/Detection");
	private final StructPublisher<Pose2d> closestCoralPose =
			visTable.getStructTopic("BestCoralPose", Pose2d.struct).publish();
	private final StructPublisher<Translation2d> closestCoralTranslation = visTable.getStructTopic(
					"BestCoralTranslation", Translation2d.struct)
			.publish();

	private Pose2d latestEstimate = new Pose2d();
	private Time latestEstimateTime = edu.wpi.first.units.Units.Seconds.of(0.0);
	protected StructPublisher<Pose2d> aprilTagPose = NetworkTableInstance.getDefault()
			.getTable("SmartDashboard/Detection/AprilTagPose")
			.getStructTopic("", Pose2d.struct)
			.publish();

	class Coral {
		Pose2d coralPose;
		Translation2d coralTranslation;
		double detectionTime;

		public Coral(Pose2d coralPose, Translation2d coralTranslation, double detectionTime) {
			this.coralPose = coralPose;
			this.coralTranslation = coralTranslation;
			this.detectionTime = detectionTime;
		}
	}

	public enum DetectionMode {
		AUTO(DetectionConstants.kAutoPipeline),
		TELE(DetectionConstants.kTelePipeline),
		DISABLED(DetectionConstants.kDisabledPipeline);

		public int index;

		private DetectionMode(int index) {
			this.index = index;
		}
	}

	public void configLimelight(LimelightConfig config) {
		this.config = config;
	}

	@Override
	public void update() {
		mStopwatch.startIfNotRunning();
		if (pipelineToSet == LimelightHelpers.getCurrentPipelineIndex(config.name)) {
			if (pipelineToSet == DetectionMode.AUTO.index) {
				Translation2d base = Drive.mInstance.getPose().getTranslation();
				RawDetection[] all = LimelightHelpers.getRawDetections(config.name);
				Translation2d bestTranslation = null;
				Pose2d bestCoralPose = null;
				double now = Timer.getFPGATimestamp();
				tracker.removeIf((coral) -> now - coral.detectionTime > 0.2);

				while (tracker.size() > 20) {
					tracker.remove(0);
				}

				for (RawDetection detection : all) {
					if (detection.classId == 0) continue;
					double tx = detection.txnc;
					double ty = detection.tync;
					Translation2d coralTranslation = calcDistToCoral(tx, ty)
							.minus(config.robotToCameraOffset.getTranslation().toTranslation2d());
					Pose2d coralPose =
							Drive.mInstance.getPose().transformBy(new Transform2d(coralTranslation, new Rotation2d()));
					if (FieldLayout.outsideField(coralPose)) {
						SmartDashboard.putBoolean("Outside Field", FieldLayout.outsideField(coralPose));
						LogUtil.recordPose2d(config.name + "Last Coral Pose Outside Field", coralPose);
						continue;
					}
					tracker.add(new Coral(coralPose, coralTranslation, now));
				}

				for (Coral coral : tracker) {
					if (bestTranslation == null
							|| bestCoralPose.getTranslation().getDistance(base)
									> coral.coralPose.getTranslation().getDistance(base)) {
						bestTranslation = coral.coralTranslation;
						bestCoralPose = coral.coralPose;
					}
				}

				if (bestCoralPose != null) {
					closestCoralPose.set(bestCoralPose);
					closestCoralTranslation.set(bestTranslation);
				}
			} else if (pipelineToSet == DetectionMode.TELE.index) {
				updateAprilTagDetection();
			}
		} else if (mStopwatch.getTime().gte(edu.wpi.first.units.Units.Seconds.of(0.5))) {
			LimelightHelpers.setPipelineIndex(config.name, (int) LimelightHelpers.getCurrentPipelineIndex(config.name));
			mResetStopwatch.resetAndStart();
			mStopwatch.reset();
		} else {
			if (mResetStopwatch.getTime().gte(edu.wpi.first.units.Units.Seconds.of(0.5))) {
				LimelightHelpers.setPipelineIndex(config.name, pipelineToSet);
				mResetStopwatch.reset();
				mStopwatch.resetAndStart();
			}
		}
	}

	@Override
	public Pose2d getCoralPose(Translation2d base) {
		Translation2d bestTranslation = null;
		Pose2d bestCoralPose = null;
		for (Coral coral : tracker) {
			if (bestTranslation == null
					|| bestCoralPose.getTranslation().getDistance(base)
							> coral.coralPose.getTranslation().getDistance(base)) {
				bestTranslation = coral.coralTranslation;
				bestCoralPose = coral.coralPose;
			}
		}
		return bestCoralPose; // will return null if no coral
	}

	@Override
	public boolean txComplete(double tx) {
		return Util.epsilonEquals(tx, 0, 4);
	}

	@Override
	public int coralCount() {
		return LimelightHelpers.getTargetCount(config.name);
	}

	@Override
	public Translation2d calcDistToCoral(double tx, double ty) {
		double totalAngleY = Units.degreesToRadians(-ty)
				- config.robotToCameraOffset.getRotation().getY();
		Distance distAwayY = config.robotToCameraOffset
				.getMeasureZ()
				.minus(DetectionConstants.kCoralRadius)
				.div(Math.tan(totalAngleY)); // robot x

		Distance distHypotenuseYToGround = BaseUnits.DistanceUnit.of(Math.hypot(
				distAwayY.in(BaseUnits.DistanceUnit),
				config.robotToCameraOffset
						.getMeasureZ()
						.minus(DetectionConstants.kCoralRadius)
						.in(BaseUnits.DistanceUnit)));

		double totalAngleX = Units.degreesToRadians(-tx)
				+ config.robotToCameraOffset.getRotation().getZ();

		Distance distAwayX = distHypotenuseYToGround.times(Math.tan(totalAngleX)); // robot y

		SmartDashboard.putNumber(config.name + "/tx", tx);
		SmartDashboard.putNumber(config.name + "/ty", ty);
		SmartDashboard.putNumber(config.name + "/Distance Away Y", distAwayY.in(edu.wpi.first.units.Units.Meters));
		SmartDashboard.putNumber(config.name + "/Distance Away X", distAwayX.in(edu.wpi.first.units.Units.Meters));
		SmartDashboard.putNumber(
				config.name + "/Distance Away Hyp ", distHypotenuseYToGround.in(edu.wpi.first.units.Units.Meters));

		return new Translation2d(distAwayY, distAwayX);
	}

	@Override
	public void setPipeline(int index) {
		pipelineToSet = index;
		LimelightHelpers.setPipelineIndex(config.name, index);
	}

	private void updateGyro() {
		Rotation2d theta = Drive.mInstance.getPose().getRotation();
		LimelightHelpers.SetRobotOrientation(config.name, theta.getDegrees(), 0, 0, 0, 0, 0);
	}

	public void updateAprilTagDetection() {
		updateGyro();
		setLatestEstimate(LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(config.name), 1);
	}

	public void setLatestEstimate(PoseEstimate poseEstimate, int minTagNum) {
		SmartDashboard.putNumber(config.name + "/Tag Count", poseEstimate.tagCount);
		SmartDashboard.putNumber(config.name + "/FGPA Timestamp", Timer.getFPGATimestamp());
		SmartDashboard.putNumber(
				config.name + "/Estimate to FGPA Timestamp", Utils.fpgaToCurrentTime(poseEstimate.timestampSeconds));
		if (poseEstimate.tagCount >= minTagNum) {
			latestEstimate = poseEstimate.pose;
			latestEstimateTime = edu.wpi.first.units.Units.Seconds.of(poseEstimate.timestampSeconds);
			aprilTagPose.set(poseEstimate.pose);
			Drive.mInstance.getGeneratedDrive();
			Drive.mInstance.addVisionUpdate(
					poseEstimate.pose,
					edu.wpi.first.units.Units.Seconds.of(poseEstimate.timestampSeconds),
					config.aprilTagVisionStdDevs);
		}
	}
}
