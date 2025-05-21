package frc.lib.drive;

import com.ctre.phoenix6.swerve.SwerveRequest;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.Pair;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.Trajectory.State;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.lib.logging.LogUtil;
import frc.lib.util.DelayedBoolean;
import frc.lib.util.FieldLayout.Level;
import frc.lib.util.SynchronousPIDF;
import frc.lib.util.Util;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.DriveConstants;
import frc.robot.subsystems.superstructure.SuperstructureConstants;
import java.util.ArrayList;
import java.util.List;

public class FollowTrajectoryCommand extends Command {
	Trajectory trajectory;
	Pose2d finalPose;
	Rotation2d targetRotation;
	Distance epsilonDist;
	Angle epsilonAngle;
	Util.Pose2dTimeInterpolable interpolable;
	Time lookaheadTime;
	DelayedBoolean atTarget;
	boolean isAuto;
	SynchronousPIDF translationController;
	SynchronousPIDF headingController;

	public FollowTrajectoryCommand(
			Trajectory trajectory,
			Distance epsilonDist,
			Angle epsilonAngle,
			Time delayTime,
			Time lookaheadTime,
			Rotation2d targetRotation,
			SynchronousPIDF translationController,
			SynchronousPIDF headingController) {
		addRequirements(Drive.mInstance);

		List<Pair<Pose2d, Time>> poseList = new ArrayList<>();
		for (State state : trajectory.getStates()) {
			poseList.add(new Pair<>(state.poseMeters, Units.Seconds.of(state.timeSeconds)));
		}
		this.trajectory = trajectory;
		this.epsilonDist = epsilonDist;
		this.epsilonAngle = epsilonAngle;
		this.lookaheadTime = lookaheadTime;
		this.targetRotation = trajectory
				.getStates()
				.get(trajectory.getStates().size() - 1)
				.poseMeters
				.getRotation()
				.plus(Rotation2d.k180deg);
		this.translationController = translationController;
		this.headingController = headingController;
		atTarget = new DelayedBoolean(Timer.getFPGATimestamp(), delayTime.in(Units.Seconds));

		LogUtil.recordTrajectory("Auto Align Traj/Trajectory", trajectory);
	}

	public FollowTrajectoryCommand(
			Trajectory trajectory,
			Distance epsilonDist,
			Angle epsilonAngle,
			Time delayTime,
			Time lookaheadTime,
			Rotation2d targetRotation) {
		this(
				trajectory,
				epsilonDist,
				epsilonAngle,
				delayTime,
				lookaheadTime,
				targetRotation,
				DriveConstants.mAutoAlignTranslationController,
				DriveConstants.mAutoAlignHeadingController);
	}

	public FollowTrajectoryCommand(
			Trajectory trajectory,
			Distance epsilonDist,
			Angle epsilonAngle,
			Time delayTime,
			Time lookaheadTime,
			SynchronousPIDF translationController,
			SynchronousPIDF headingController) {
		this(
				trajectory,
				epsilonDist,
				epsilonAngle,
				delayTime,
				lookaheadTime,
				trajectory
						.getStates()
						.get(trajectory.getStates().size() - 1)
						.poseMeters
						.getRotation(),
				translationController,
				headingController);
	}

	public FollowTrajectoryCommand(
			Trajectory trajectory,
			Level level,
			SynchronousPIDF translationController,
			SynchronousPIDF headingController) {
		this(
				trajectory,
				SuperstructureConstants.getAutoAlignScoringDistanceEpsilon(level),
				SuperstructureConstants.getAutoAlignScoringAngleEpsilon(level),
				SuperstructureConstants.getAutoAlignScoringDelay(level),
				SuperstructureConstants.getAutoAlignLookaheadTime(level),
				trajectory
						.getStates()
						.get(trajectory.getStates().size() - 1)
						.poseMeters
						.getRotation(),
				translationController,
				headingController);
	}

	public FollowTrajectoryCommand(Trajectory trajectory, Level level) {
		this(
				trajectory,
				SuperstructureConstants.getAutoAlignScoringDistanceEpsilon(level),
				SuperstructureConstants.getAutoAlignScoringAngleEpsilon(level),
				SuperstructureConstants.getAutoAlignScoringDelay(level),
				SuperstructureConstants.getAutoAlignLookaheadTime(level),
				trajectory
						.getStates()
						.get(trajectory.getStates().size() - 1)
						.poseMeters
						.getRotation(),
				DriveConstants.mAutoAlignTranslationController,
				DriveConstants.mAutoAlignHeadingController);
	}

	public FollowTrajectoryCommand(
			Trajectory trajectory,
			Time delayTime,
			Time lookaheadTime,
			Rotation2d targetRotation,
			SynchronousPIDF translationController,
			SynchronousPIDF headingController) {
		this(
				trajectory,
				Units.Centimeters.of(4.0),
				Units.Degrees.of(0.8),
				delayTime,
				lookaheadTime,
				targetRotation,
				translationController,
				headingController);
	}

	public FollowTrajectoryCommand(
			Trajectory trajectory,
			Time delayTime,
			Time lookaheadTime,
			SynchronousPIDF translationController,
			SynchronousPIDF headingController) {
		this(
				trajectory,
				delayTime,
				lookaheadTime,
				trajectory
						.getStates()
						.get(trajectory.getStates().size() - 1)
						.poseMeters
						.getRotation(),
				translationController,
				headingController);
	}

	public FollowTrajectoryCommand(
			Trajectory trajectory,
			Time delayTime,
			Rotation2d targetRotation,
			SynchronousPIDF translationController,
			SynchronousPIDF headingController) {
		this(trajectory, delayTime, Units.Seconds.of(0.35), targetRotation, translationController, headingController);
	}

	public FollowTrajectoryCommand(
			Trajectory trajectory, Rotation2d targetRotation, SynchronousPIDF translationController) {
		this(
				trajectory,
				Units.Seconds.of(0.04),
				targetRotation,
				translationController,
				DriveConstants.mAutoAlignHeadingController);
	}

	public FollowTrajectoryCommand(Trajectory trajectory, Rotation2d targetRotation) {
		this(
				trajectory,
				Units.Seconds.of(0.04),
				targetRotation,
				DriveConstants.mAutoAlignTranslationController,
				DriveConstants.mAutoAlignHeadingController);
	}

	public FollowTrajectoryCommand(Trajectory trajectory, SynchronousPIDF translationController) {
		this(
				trajectory,
				Units.Seconds.of(0.04),
				trajectory
						.getStates()
						.get(trajectory.getStates().size() - 1)
						.poseMeters
						.getRotation(),
				translationController,
				DriveConstants.mAutoAlignHeadingController);
	}

	public FollowTrajectoryCommand(Trajectory trajectory) {
		this(
				trajectory,
				trajectory
						.getStates()
						.get(trajectory.getStates().size() - 1)
						.poseMeters
						.getRotation());
	}

	@Override
	public void initialize() {
		interpolable = new Util.Pose2dTimeInterpolable(
				trajectory, Drive.mInstance.getPose().getRotation(), targetRotation);
		finalPose = trajectory
				.getStates()
				.get(trajectory.getStates().size() - 1)
				.poseMeters
				.plus(new Transform2d(new Translation2d(), Rotation2d.k180deg));
		LogUtil.recordPose2d("Auto Align Traj/Target Pose", finalPose);
	}

	@Override
	public void execute() {
		Time currentTime = interpolable.getTimeFromPose(Drive.mInstance.getPose());
		interpolable.clearStatesBeforeTime(currentTime);
		Pose2d lookaheadPose = interpolable.getPoseFromTime(currentTime.plus(lookaheadTime));
		LogUtil.recordPose2d("Auto Align Traj/Pose To Pursue", lookaheadPose);
		Drive.mInstance.setSwerveRequest(
				DriveConstants.getPIDToPoseRequestUpdater(lookaheadPose).apply(DriveConstants.PIDToPoseRequest));
	}

	@Override
	public void end(boolean interrupted) {
		Drive.mInstance.setSwerveRequest(new SwerveRequest.ApplyRobotSpeeds());
	}

	public boolean atEndPose() {
		Pose2d currentPose = Drive.mInstance.getPose();

		boolean complete = atTarget.update(
				Timer.getFPGATimestamp(),
				currentPose.getTranslation().getDistance(finalPose.getTranslation()) < epsilonDist.in(Units.Meters)
						&& MathUtil.angleModulus(Math.abs(currentPose
										.getRotation()
										.minus(finalPose.getRotation())
										.getRadians()))
								< epsilonAngle.in(Units.Radians));

		SmartDashboard.putBoolean("Auto Align Traj/Completed", complete);
		SmartDashboard.putBoolean(
				"Auto Align Traj/Translation Completed",
				currentPose.getTranslation().getDistance(finalPose.getTranslation()) < epsilonDist.in(Units.Meters));
		SmartDashboard.putBoolean(
				"Auto Align Traj/Rotation Completed",
				MathUtil.angleModulus(Math.abs(currentPose
								.getRotation()
								.minus(finalPose.getRotation())
								.getRadians()))
						< epsilonAngle.in(Units.Radians));
		SmartDashboard.putNumber(
				"Auto Align Traj/Distance Away Inches",
				currentPose.getTranslation().getDistance(finalPose.getTranslation()) * 39.37);

		return complete;
	}

	@Override
	public boolean isFinished() {
		return atEndPose();
	}

	public Distance distanceFromEnd() {
		return Units.Meters.of(Drive.mInstance
				.getPose()
				.getTranslation()
				.minus(finalPose.getTranslation())
				.getNorm());
	}
}
