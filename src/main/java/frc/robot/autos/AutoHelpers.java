package frc.robot.autos;

import choreo.auto.AutoFactory;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.DeferredCommand;
import frc.lib.drive.FollowSyncedPIDToPose;
import frc.lib.drive.FollowSyncedTrajectory;
import frc.lib.drive.TrajectoryHelpers;
import frc.lib.util.FieldLayout;
import frc.lib.util.FieldLayout.Branch;
import frc.lib.util.FieldLayout.Level;
import frc.lib.util.Stopwatch;
import frc.robot.RobotConstants;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.superstructure.Superstructure;
import java.util.Set;

public class AutoHelpers {
	public static Stopwatch stopwatch = new Stopwatch();

	public static void bindEventMarkers(AutoFactory mAutoFactory) {
		Superstructure s = Superstructure.mInstance;
		mAutoFactory.bind("intake", s.coralIntakeToHold());
		mAutoFactory.bind("tuck", s.tuck());
		mAutoFactory.bind("human", s.stationIntakePrep());
	}

	public static String getName(AutoModeSelector mAutoModeSelector) {
		return mAutoModeSelector.getSelectedCommand().getName();
	}

	public static Command getAutoScorePathFromDrivePose(Branch wantedBranch, Level level) {
		return new DeferredCommand(
				() -> {
					Pose2d scoringPose = FieldLayout.handleAllianceFlip(
							FieldLayout.getCoralScoringPose(wantedBranch), RobotConstants.isRedAlliance);
					return new FollowSyncedPIDToPose(scoringPose, level);
				},
				Set.of(Drive.mInstance));
	}

	public static Command getAutoScoreTrajectoryFromDrivePose(Branch wantedBranch, Level level) {
		return new DeferredCommand(
				() -> {
					Pose2d scoringPose = FieldLayout.handleAllianceFlip(
							FieldLayout.getCoralScoringPose(wantedBranch), RobotConstants.isRedAlliance);

					Trajectory traj = TrajectoryHelpers.generateGamepieceBasedTrajectoryFromDrive(
							scoringPose, Units.Degrees.of(0.0), level);

					return new FollowSyncedTrajectory(traj);
				},
				Set.of(Drive.mInstance));
	}

	public static Command getAutoScorePathWithStartPose(Branch wantedBranch, Level level, Pose2d startPose) {
		Pose2d scoringPose = FieldLayout.handleAllianceFlip(
				FieldLayout.getCoralScoringPose(wantedBranch), RobotConstants.isRedAlliance);

		return new FollowSyncedPIDToPose(scoringPose, level);
	}

	public static Command getPrepAndScoreForLevel(Level level) {
		Superstructure s = Superstructure.mInstance;
		Command scoreCommand =
				switch (level) {
					case L2 -> s.L2ScoreInAuto();
					case L3 -> s.L3ScoreInAuto();
					case L4 -> s.L4ScoreInAuto();
					default -> s.L4ScoreInAuto();
				};
		return scoreCommand.asProxy();
	}

	public static Command resetPoseIfWithoutEstimate(Pose2d pose) {
		return Commands.runOnce(() -> Drive.mInstance.resetPose(pose));
	}
}
