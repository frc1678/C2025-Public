package frc.lib.drive;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;
import frc.lib.util.FieldLayout.Level;
import frc.lib.util.SynchronousPIDF;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.DriveConstants;
import frc.robot.subsystems.superstructure.Superstructure;

/**
 * Used to sync a PIDToPoseCommand with a superstructure action (like a scoring sequence).
 * Also adds profiling to the PIDToPoseCommand, allowing us to control our slow downs to avoid tippiness and other issues.
 * An example of where this would be used is a net auto align, where our CG gets very high.
 */
public class FollowSyncedPIDToPose extends PIDToPoseCommand {
	private final SynchronousPIDF tippyTranslationController;
	private final SynchronousPIDF tippyHeadingController;
	private final Distance distanceToStartSlowing;
	private final Distance distanceToRaiseElevator;
	private final LinearVelocity velToRaiseElevator;

	public FollowSyncedPIDToPose(
			Pose2d finalPose, Level level, SynchronousPIDF translationController, SynchronousPIDF headingController) {
		super(finalPose, level, translationController, headingController);

		Superstructure.mInstance.setSuperstructureDone(false);
		Superstructure.mInstance.setDriveReady(false);
		Superstructure.mInstance.setReadyToRaiseElevator(false);

		tippyTranslationController = DriveConstants.getTippyTranslationControllerForLevel(level);
		tippyHeadingController = DriveConstants.getTippyHeadingControllerForLevel(level);
		distanceToStartSlowing = DriveConstants.getDistanceToStartSlowingForLevel(level);
		distanceToRaiseElevator = DriveConstants.getDistanceToRaiseElevatorForLevel(level);
		velToRaiseElevator = DriveConstants.getVelocityToRaiseForLevel(level);
	}

	public FollowSyncedPIDToPose(Pose2d finalPose, Level level, SynchronousPIDF translationController) {
		super(finalPose, level, translationController);

		Superstructure.mInstance.setSuperstructureDone(false);
		Superstructure.mInstance.setDriveReady(false);
		Superstructure.mInstance.setReadyToRaiseElevator(false);

		tippyTranslationController = DriveConstants.getTippyTranslationControllerForLevel(level);
		tippyHeadingController = DriveConstants.getTippyHeadingControllerForLevel(level);
		distanceToStartSlowing = DriveConstants.getDistanceToStartSlowingForLevel(level);
		distanceToRaiseElevator = DriveConstants.getDistanceToRaiseElevatorForLevel(level);
		velToRaiseElevator = DriveConstants.getVelocityToRaiseForLevel(level);
	}

	public FollowSyncedPIDToPose(Pose2d finalPose, Level level) {
		super(finalPose, level);

		Superstructure.mInstance.setSuperstructureDone(false);
		Superstructure.mInstance.setDriveReady(false);
		Superstructure.mInstance.setReadyToRaiseElevator(false);

		tippyTranslationController = DriveConstants.getTippyTranslationControllerForLevel(level);
		tippyHeadingController = DriveConstants.getTippyHeadingControllerForLevel(level);
		distanceToStartSlowing = DriveConstants.getDistanceToStartSlowingForLevel(level);
		distanceToRaiseElevator = DriveConstants.getDistanceToRaiseElevatorForLevel(level);
		velToRaiseElevator = DriveConstants.getVelocityToRaiseForLevel(level);
	}

	public FollowSyncedPIDToPose(Pose2d rawEndPose, Level level, boolean diffParam) {
		super(rawEndPose, level, diffParam);

		Superstructure.mInstance.setSuperstructureDone(false);
		Superstructure.mInstance.setDriveReady(false);
		Superstructure.mInstance.setReadyToRaiseElevator(false);

		tippyTranslationController = DriveConstants.getTippyTranslationControllerForLevel(level);
		tippyHeadingController = DriveConstants.getTippyHeadingControllerForLevel(level);
		distanceToStartSlowing = DriveConstants.getDistanceToStartSlowingForLevel(level);
		distanceToRaiseElevator = DriveConstants.getDistanceToRaiseElevatorForLevel(level);
		velToRaiseElevator = DriveConstants.getVelocityToRaiseForLevel(level);
	}

	public boolean driveDone() {
		return super.isFinished();
	}

	public boolean closeEnoughToRaiseElevator() {
		return distanceFromEnd().lte(distanceToRaiseElevator);
	}

	public boolean slowEnoughToRaiseElevator() {
		return Math.hypot(
						Drive.mInstance.getState().Speeds.vxMetersPerSecond,
						Drive.mInstance.getState().Speeds.vxMetersPerSecond)
				< velToRaiseElevator.in(Units.MetersPerSecond);
	}

	@Override
	public void execute() {
		boolean shouldStartSlowing = distanceFromEnd().lte(distanceToStartSlowing);
		if (shouldStartSlowing) {
			Drive.mInstance.setSwerveRequest(DriveConstants.getPIDToPoseRequestUpdater(
							finalPose, tippyTranslationController, tippyHeadingController)
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
