package frc.lib.drive;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Time;
import frc.lib.util.FieldLayout.Level;
import frc.lib.util.SynchronousPIDF;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.DriveConstants;
import frc.robot.subsystems.superstructure.Superstructure;

/**
 * Used to sync a PIDToPoseCommand with a superstructure action (like a scoring sequence).
 * Not used as often as FollowSyncedPIDToPose--for when you aren't sure you'll be able to go in a straight line.
 * An example of that would be our trajectory to E in the Detection FCDE auto.
 */
public class FollowSyncedTrajectory extends FollowTrajectoryCommand {
	public FollowSyncedTrajectory(
			Trajectory trajectory,
			Distance epsilonDist,
			Angle epsilonAngle,
			Time delayTime,
			Time lookaheadTime,
			Rotation2d targetRotation,
			boolean isAuto) {
		super(trajectory, epsilonDist, epsilonAngle, delayTime, lookaheadTime, targetRotation);
		Superstructure.mInstance.setSuperstructureDone(false);
		Superstructure.mInstance.setDriveReady(false);
	}

	public FollowSyncedTrajectory(Trajectory trajectory, Level level) {
		super(trajectory, level);
		Superstructure.mInstance.setSuperstructureDone(false);
		Superstructure.mInstance.setDriveReady(false);
	}

	public FollowSyncedTrajectory(Trajectory trajectory, Rotation2d targetRotation) {
		super(trajectory, targetRotation);
		Superstructure.mInstance.setSuperstructureDone(false);
		Superstructure.mInstance.setDriveReady(false);
	}

	public FollowSyncedTrajectory(Trajectory trajectory, SynchronousPIDF translationController) {
		super(trajectory, translationController);
		Superstructure.mInstance.setSuperstructureDone(false);
		Superstructure.mInstance.setDriveReady(false);
	}

	public FollowSyncedTrajectory(Trajectory trajectory) {
		super(
				trajectory,
				trajectory
						.getStates()
						.get(trajectory.getStates().size() - 1)
						.poseMeters
						.getRotation());
		Superstructure.mInstance.setSuperstructureDone(false);
		Superstructure.mInstance.setDriveReady(false);
	}

	public boolean driveDone() {
		return super.isFinished();
	}

	public boolean closeEnoughToRaiseElevator() {
		return distanceFromEnd().lte(DriveConstants.distanceToRaiseElevatorL4);
	}

	public boolean slowEnoughToRaiseElevator() {
		return Math.hypot(
						Drive.mInstance.getState().Speeds.vxMetersPerSecond,
						Drive.mInstance.getState().Speeds.vxMetersPerSecond)
				< DriveConstants.kMaxSpeedTippy.times(1.1).in(Units.MetersPerSecond);
	}

	@Override
	public void execute() {

		boolean shouldStartSlowing = distanceFromEnd().lte(DriveConstants.distanceToStartSlowingL4);
		if (shouldStartSlowing) {
			Drive.mInstance.setSwerveRequest(DriveConstants.getPIDToPoseRequestUpdater(
							finalPose,
							DriveConstants.mAutoAlignTippyTranslationController,
							DriveConstants.mAutoAlignTippyHeadingController)
					.apply(DriveConstants.PIDToPoseRequest));
		} else {
			super.execute();
		}
		Superstructure.mInstance.setDriveReady(driveDone());
		boolean closeEnoughToRaiseElevator = closeEnoughToRaiseElevator();
		boolean slowEnoughToRaiseElevator = slowEnoughToRaiseElevator();
		Superstructure.mInstance.setReadyToRaiseElevator(closeEnoughToRaiseElevator && slowEnoughToRaiseElevator);
	}

	@Override
	public boolean isFinished() {
		return Superstructure.mInstance.getSuperstructureDone();
	}
}
