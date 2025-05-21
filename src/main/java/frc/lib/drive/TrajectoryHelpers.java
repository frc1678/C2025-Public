package frc.lib.drive;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.Trajectory.State;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import frc.lib.logging.LogUtil;
import frc.lib.util.FieldLayout.Level;
import frc.lib.util.Util;
import frc.robot.subsystems.detection.Detection;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.DriveConstants;
import frc.robot.subsystems.superstructure.SuperstructureConstants;
import java.util.ArrayList;
import java.util.List;

public class TrajectoryHelpers {
	/**
	 * Uses an arctan to calculate the angle between your robot and your goal translation.
	 *
	 * @param translation The translation component of your target pose.
	 * @return The angle between your robot's position and the goal position. Used for
	 * calculating the initial tangent of a path.
	 */
	public static Rotation2d angleToScore(Translation2d translation) {
		Translation2d scoringTranslation = translation;
		return scoringTranslation
				.minus(Drive.mInstance.getPose().getTranslation())
				.getAngle();
	}

	/**
	 * Uses an arctan to calculate the angle between your robot and your goal translation.
	 *
	 * @param translation The translation component of your target pose.
	 * @return The angle between your robot's position and the goal position. Used for
	 * calculating the initial tangent of a path.
	 */
	public static Rotation2d angleToScoreWithStartPose(Translation2d translation, Pose2d pose) {
		Translation2d scoringTranslation = translation;
		return scoringTranslation.minus(pose.getTranslation()).getAngle();
	}

	/**
	 * Compares your wanted final tangent to where you are on the field, adjusting that
	 * tangent to better work with your path.
	 *
	 * @param translation The translation component of your target pose.
	 * @param idealAngle The ideal approach angle.
	 * @return The final tangent that you should use for your path.
	 */
	public static Rotation2d angleToApproach(Translation2d translation, Rotation2d idealAngle, Angle deadband) {
		Rotation2d currentAngle = angleToScore(translation);
		double currentDegrees = currentAngle.getDegrees();
		double targetDegrees = idealAngle.getDegrees();

		if (idealAngle.getMeasure().isEquivalent(Units.Degrees.of(180))
				&& currentAngle.getMeasure().lte(Units.Degrees.of(0))) currentDegrees += 360.0;

		return Util.epsilonEquals(currentDegrees, targetDegrees, deadband.in(Units.Degrees))
				? Rotation2d.fromDegrees(currentDegrees)
				: idealAngle.plus(Rotation2d.fromDegrees(
						Math.signum(currentDegrees - targetDegrees) * deadband.in(Units.Degrees)));
	}

	/**
	 * Generates a trajectory for your drivetrain to follow based on a target pose.
	 *
	 * @param targetPose The wanted ending pose (x, y, heading).
	 * @return The OTF trajectory to follow based on your current pose and target pose.
	 */
	public static Trajectory generateTrajectoryFromDrive(Pose2d targetPose) {
		List<Pose2d> waypoints = new ArrayList<>();
		waypoints.add(
				new Pose2d(Drive.mInstance.getPose().getTranslation(), angleToScore(targetPose.getTranslation())));
		waypoints.add(targetPose);
		return TrajectoryGenerator.generateTrajectory(waypoints, DriveConstants.getTrajectoryConfig());
	}

	/**
	 * Generates a trajectory for your drivetrain to follow based on a target pose.
	 *
	 * @param targetPose The wanted ending pose (x, y, heading).
	 * @return The OTF trajectory to follow based on your current pose and target pose.
	 */
	public static Trajectory generateTrajectoryFromDriveWithTangent(Pose2d targetPose, Translation2d coralMark) {
		List<Pose2d> waypoints = new ArrayList<>();
		waypoints.add(new Pose2d(Drive.mInstance.getPose().getTranslation(), angleToScore(coralMark)));
		waypoints.add(targetPose);
		return TrajectoryGenerator.generateTrajectory(waypoints, DriveConstants.getTrajectoryConfig());
	}

	/**
	 * Generates a trajectory for your drivetrain to follow based on a target pose.
	 *
	 * @param targetPose The wanted ending pose (x, y, heading).
	 * @return The OTF trajectory to follow based on your current pose and target pose.
	 */
	public static Trajectory generateTrajectory(Pose2d targetPose, Pose2d startPose) {
		List<Pose2d> waypoints = new ArrayList<>();
		waypoints.add(new Pose2d(
				startPose.getTranslation(), angleToScoreWithStartPose(targetPose.getTranslation(), startPose)));
		waypoints.add(targetPose);
		return TrajectoryGenerator.generateTrajectory(waypoints, DriveConstants.getTrajectoryConfig());
	}

	/**
	 * Generates a trajectory for your drivetrain to follow based on the target pose of *the end of a gamepiece*.
	 *
	 * @param gamepieceTargetPose The wanted ending pose (x, y, heading) of *the end of a gamepiece*.
	 * @param idealApproachDeadband The allowed difference (+-) from the ideal heading that the trajectory is allowed to generate with.
	 * @param level The wanted CORAL level (L1, L2, L3, L4).
	 * @return The OTF trajectory to follow based on your current pose and wanted ending pose.
	 */
	public static Trajectory generateGamepieceBasedTrajectory(
			Pose2d gamepieceTargetPose, Angle idealApproachDeadband, Level level, Pose2d startingPose) {
		LogUtil.recordPose2d("Auto Align Traj/Gamepiece Target Pose", gamepieceTargetPose);
		Pose2d transformedTargetPose = getDriveTargetPose(gamepieceTargetPose, idealApproachDeadband, level);

		if (Util.epsilonEquals(
				startingPose.getTranslation(),
				transformedTargetPose.getTranslation(),
				Math.sqrt(Units.Centimeters.of(4.0).in(Units.Meters)))) {
			Trajectory smallCorrectionTrajectory = new Trajectory();
			for (int i = 0; i < 10; ++i) {
				State state = new State();
				state.poseMeters = transformedTargetPose;
				state.timeSeconds = (double) i * 0.05;
				smallCorrectionTrajectory.getStates().add(state);
			}
			return smallCorrectionTrajectory;
		}

		return generateTrajectory(transformedTargetPose, startingPose);
	}

	/**
	 * Generates a trajectory for your drivetrain to follow based on the target pose of *the end of a gamepiece*.
	 *
	 * @param gamepieceTargetPose The wanted ending pose (x, y, heading) of *the end of a gamepiece*.
	 * @param idealApproachDeadband The allowed difference (+-) from the ideal heading that the trajectory is allowed to generate with.
	 * @param level The wanted CORAL level (L1, L2, L3, L4).
	 * @return The OTF trajectory to follow based on your current pose and wanted ending pose.
	 */
	public static Trajectory generateGamepieceBasedTrajectoryFromDrive(
			Pose2d gamepieceTargetPose, Angle idealApproachDeadband, Level level) {
		LogUtil.recordPose2d("Auto Align Traj/Gamepiece Target Pose", gamepieceTargetPose);
		Pose2d transformedTargetPose = getDriveTargetPose(gamepieceTargetPose, idealApproachDeadband, level);

		if (Util.epsilonEquals(
				Drive.mInstance.getPose().getTranslation(),
				transformedTargetPose.getTranslation(),
				Math.sqrt(Units.Centimeters.of(4.0).in(Units.Meters)))) {
			Trajectory smallCorrectionTrajectory = new Trajectory();
			for (int i = 0; i < 10; ++i) {
				State state = new State();
				state.poseMeters = transformedTargetPose;
				state.timeSeconds = (double) i * 0.05;
				smallCorrectionTrajectory.getStates().add(state);
			}
			return smallCorrectionTrajectory;
		}

		return generateTrajectoryFromDrive(transformedTargetPose);
	}

	/**
	 * Generates a trajectory for your drivetrain to follow based on the target pose of *the end of a gamepiece*.
	 *
	 * @param gamepieceTargetPose The wanted ending pose (x, y, heading) of *the end of a gamepiece*.
	 * @param idealApproachDeadband The allowed difference (+-) from the ideal heading that the trajectory is allowed to generate with.
	 * @param level The wanted CORAL level (L1, L2, L3, L4).
	 * @return The OTF trajectory to follow based on your current pose and wanted ending pose.
	 */
	public static Trajectory generateGamepieceBasedTrajectoryFromDriveWithTangent(
			Pose2d gamepieceTargetPose, Angle idealApproachDeadband, Level level, Translation2d translationForTangent) {
		LogUtil.recordPose2d("Auto Align Traj/Gamepiece Target Pose", gamepieceTargetPose);
		Pose2d transformedTargetPose = getDriveTargetPose(gamepieceTargetPose, idealApproachDeadband, level);

		if (Util.epsilonEquals(
				Drive.mInstance.getPose().getTranslation(),
				transformedTargetPose.getTranslation(),
				Math.sqrt(Units.Centimeters.of(4.0).in(Units.Meters)))) {
			Trajectory smallCorrectionTrajectory = new Trajectory();
			for (int i = 0; i < 10; ++i) {
				State state = new State();
				state.poseMeters = transformedTargetPose;
				state.timeSeconds = (double) i * 0.05;
				smallCorrectionTrajectory.getStates().add(state);
			}
			return smallCorrectionTrajectory;
		}

		return generateTrajectoryFromDriveWithTangent(transformedTargetPose, translationForTangent);
	}

	/**
	 * Due to the location this is being applied at, the transform ends up being applied the same way as the opposite method.
	 * This is because these transforms are robot-relative.
	 *
	 * @param wantedFinalPose Wanted final pose of *a gamepiece in the end effector*.
	 * @param offsetLength The distance that the gamepiece is from the center of the robot.
	 * @return The pose the center of your robot will be at.
	 */
	public static Pose2d transformWantedGamepieceToDrivePose(Pose2d wantedFinalPose, Distance offsetLength) {
		Rotation2d r = wantedFinalPose.getRotation();
		Distance x = wantedFinalPose.getMeasureX().minus(offsetLength.times(r.getCos()));
		Distance y = wantedFinalPose.getMeasureY().minus(offsetLength.times(r.getSin()));
		return new Pose2d(x, y, r);
	}

	/**
	 * Due to the location this is being applied at, the transform ends up being applied the same way as the opposite method.
	 * This is because these transforms are robot-relative.
	 *
	 * @param wantedFinalPose Wanted final pose of *the drivetrain*.
	 * @param offsetLength The distance that the center of the robot is from a gamepiece in the end effector.
	 * @return The pose the center of your robot will be at.
	 */
	public static Pose2d transformWantedDriveToGamepiecePose(Pose2d wantedFinalPose, Distance offsetLength) {
		Rotation2d r = wantedFinalPose.getRotation();
		Distance x = wantedFinalPose.getMeasureX().minus(offsetLength.times(r.getCos()));
		Distance y = wantedFinalPose.getMeasureY().minus(offsetLength.times(r.getSin()));
		return new Pose2d(x, y, r);
	}

	/**
	 * Gets the wanted pose for the center of your robot from the End Effector.
	 *
	 * @param endEffectorPose The pose to transform to a robot pose.
	 * @param idealApproachDeadband The allowed difference (+-) from the ideal heading that the trajectory is allowed to generate with.
	 * @param level The wanted gamepiece level (L1, L2, L3, L4).
	 * @return The transformed pose to pass to Drive.
	 */
	public static Pose2d getDriveTargetPose(Pose2d endEffectorPose, Angle idealApproachDeadband, Level level) {
		Rotation2d angle =
				angleToApproach(endEffectorPose.getTranslation(), endEffectorPose.getRotation(), idealApproachDeadband);
		Pose2d transformedTargetPose = transformWantedGamepieceToDrivePose(
				new Pose2d(endEffectorPose.getTranslation(), angle),
				SuperstructureConstants.kElevatorCenterOffset.plus(
						SuperstructureConstants.getGamepieceOffsetFactor(level)));

		return transformedTargetPose;
	}

	/**
	 * Gets the expected pose for the end of a gamepiece based on a drivetrain pose
	 *
	 * @param drivePose The pose of the drivetrain.
	 * @param level The wanted gamepiece level (L1, L2, L3, L4).
	 * @return Expected pose for a gamepiece in the end effector
	 */
	public static Pose2d getGamepieceTargetPose(Pose2d drivePose, Level level) {
		return transformWantedDriveToGamepiecePose(drivePose, SuperstructureConstants.getGamepieceOffsetFactor(level));
	}

	/**
	 * Gets the expected pose for the end of a gamepiece based on the current drivetrain pose
	 *
	 * @param level The wanted gamepiece level (L1, L2, L3, L4).
	 * @return Expected pose for a gamepiece in the end effector
	 */
	public static Pose2d getGamepieceCurrentPose(Level level) {
		return getGamepieceTargetPose(Drive.mInstance.getPose(), level);
	}

	public static Trajectory getCoralDetectionTrajectory() {
		if (Detection.mInstance.hasCoral()) {
			Translation2d finalTranslation = Detection.mInstance.getCoralPose().getTranslation();
			Rotation2d finalRotation = finalTranslation
					.minus(Drive.mInstance.getPose().getTranslation())
					.getAngle();
			return generateTrajectory(new Pose2d(finalTranslation, finalRotation), Drive.mInstance.getPose());
		}
		return null;
	}
}
