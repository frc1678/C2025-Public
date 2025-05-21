package frc.lib.drive;

import choreo.auto.AutoTrajectory;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.lib.util.FieldLayout;
import frc.lib.util.MovingAveragePose2d;
import frc.lib.util.Stopwatch;
import frc.lib.util.Util;
import frc.robot.autos.AutoConstants;
import frc.robot.autos.AutoConstants.AutoType;
import frc.robot.subsystems.detection.Detection;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.superstructure.Superstructure;

/**
 * PIDToPoseCommand that updates if there is a detection.
 * If there is never a detection, it goes to a default, predetermined pose.
 */
public class DetectionPIDToPoseCommand extends PIDToPoseCommand {
	private final Pose2d defaultFinalPose;
	private Stopwatch timer = new Stopwatch();
	private Stopwatch debounce = new Stopwatch();

	private AutoType side;
	private boolean firstUpdate; // stores when the final pose was first updated

	private MovingAveragePose2d movingAveragePose2d =
			new MovingAveragePose2d(15); // more likely to trust new poses but still keeps track of a few older ones

	public DetectionPIDToPoseCommand(Pose2d defaultFinalPose, AutoType side) {
		super(
				defaultFinalPose,
				AutoConstants.getDetectionTranslationController(),
				AutoConstants.getDetectionHeadingController());
		this.defaultFinalPose = defaultFinalPose;
		this.side = side;
		this.epsilonDist = Units.Inches.of(4.0);
		this.epsilonAngle = Units.Degrees.of(4.0);
	}

	public DetectionPIDToPoseCommand(AutoTrajectory trajectory, AutoType side) {
		this(trajectory.getFinalPose().get(), side);
	}

	@Override
	public void initialize() {
		firstUpdate = false;
		finalPose = defaultFinalPose;
		movingAveragePose2d.clear();
		timer.startIfNotRunning();
		super.initialize();
	}

	@Override
	public void execute() {
		if (Detection.mInstance.hasCoral()) {
			debounce.startIfNotRunning();
			if (debounce.getTime().gte(Units.Milliseconds.of(40.0)) && !firstUpdate) {
				firstUpdate = true;
				debounce.reset();
			}

			Rotation2d rot = Detection.mInstance
					.getCoralTranslationAndPoint()
					.getTranslation()
					.minus(Drive.mInstance.getPose().getTranslation())
					.getAngle();
			Translation2d finalTranslation = Detection.mInstance
					.getCoralTranslationAndPoint()
					.getTranslation()
					.minus(new Translation2d(edu.wpi.first.math.util.Units.feetToMeters(0.0), rot));

			Pose2d newPose = new Pose2d(
					finalTranslation,
					Detection.mInstance.getCoralTranslationAndPoint().getRotation());

			if (side == AutoType.MARK) {
				if (firstUpdate) {
					movingAveragePose2d.add(newPose);
					finalPose = movingAveragePose2d.getAverage();
				}
			} else {
				if (firstUpdate
						&& FieldLayout.getIsInZone(newPose.getTranslation(), side)
						&& !FieldLayout.nearMark(newPose)
						&& Util.epsilonEquals(
								Drive.mInstance.getPose().getRotation().getDegrees(),
								Detection.mInstance
										.getCoralTranslationAndPoint()
										.getRotation()
										.getDegrees(),
								60)
						&& !Util.epsilonEquals(
								Drive.mInstance.getPose().getTranslation(),
								finalPose.getTranslation(),
								Units.Feet.of(0.0))) {
					movingAveragePose2d.add(newPose);
					finalPose = movingAveragePose2d.getAverage();
					SmartDashboard.putNumber("Detection PID/Last Accepted Update", Timer.getFPGATimestamp());
				}
			}
		}
		super.execute();
	}

	@Override
	public void end(boolean interrupted) {
		timer.reset();
		debounce.reset();
		super.end(interrupted);
	}

	@Override
	public boolean isFinished() {
		return super.isFinished()
				|| Superstructure.indexerBreak.getDebouncedIfReal()
				|| timer.getTime().gte(Units.Seconds.of(3.5));
	}
}
