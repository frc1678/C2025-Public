package frc.robot.autos;

import choreo.auto.AutoFactory;
import choreo.auto.AutoRoutine;
import choreo.auto.AutoTrajectory;
import com.ctre.phoenix6.swerve.SwerveRequest;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import frc.lib.drive.DetectionPIDToPoseCommand;
import frc.lib.drive.PIDToPoseCommand;
import frc.lib.logging.LogUtil;
import frc.lib.util.FieldLayout.Branch;
import frc.lib.util.FieldLayout.Level;
import frc.lib.util.Stopwatch;
import frc.robot.RobotConstants;
import frc.robot.autos.AutoConstants.AutoType;
import frc.robot.subsystems.coraldeploy.CoralDeploy;
import frc.robot.subsystems.detection.Detection;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.DriveConstants;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.elevator.ElevatorConstants;
import frc.robot.subsystems.endeffector.EndEffector;
import frc.robot.subsystems.superstructure.Superstructure;
import frc.robot.subsystems.superstructure.SuperstructureConstants;
import java.util.Arrays;
import java.util.List;
import java.util.Set;

public class AutoModeBase {
	private static AutoRoutine routine;
	private static Stopwatch stopwatch = new Stopwatch();
	private static AutoType side;

	public AutoModeBase(AutoFactory factory, String name) {
		routine = factory.newRoutine(name);
	}

	public AutoModeBase(AutoFactory factory, String name, AutoType side) {
		this(factory, name);
		AutoModeBase.side = side;
	}

	/**
	 * @return Trajectory from choreo
	 */
	public AutoTrajectory trajectory(String name) {
		return routine.trajectory(name);
	}

	public AutoTrajectory trajectory(String name, int index) {
		return routine.trajectory(name, index);
	}

	/**
	 * Runs an accuracy-based command for choreo following
	 *
	 * @param trajectory
	 * @param timeout
	 */
	public static Command cmdWithRotationAccuracy(AutoTrajectory trajectory, Time timeout) {
		return Commands.defer(
						() -> new FunctionalCommand(
								trajectory.cmd()::initialize,
								trajectory.cmd()::execute,
								trajectory.cmd()::end,
								() -> rotationIsFinished(trajectory)),
						Set.of(Drive.mInstance))
				.beforeStarting(() -> Superstructure.mInstance.setDriveReady(false))
				.withTimeout(trajectory.getRawTrajectory().getTotalTime() + timeout.in(Units.Seconds));
	}

	public static Command cmdWithRotationAccuracy(AutoTrajectory trajectory) {
		return cmdWithRotationAccuracy(trajectory, AutoConstants.kDefaultTrajectoryTimeout);
	}

	/**
	 * Runs an accuracy-based command for choreo following
	 *
	 * @param trajectory
	 * @param timeout
	 */
	public static Command cmdWithAccuracy(AutoTrajectory trajectory, Time timeout, Distance epsilonDist) {
		return Commands.defer(
						() -> new FunctionalCommand(
								trajectory.cmd()::initialize,
								trajectory.cmd()::execute,
								trajectory.cmd()::end,
								() -> isFinished(trajectory, epsilonDist)),
						Set.of(Drive.mInstance))
				.beforeStarting(() -> Superstructure.mInstance.setDriveReady(false))
				.withTimeout(trajectory.getRawTrajectory().getTotalTime() + timeout.in(Units.Seconds));
	}

	/**
	 * Returns an accuracy-based command for choreo following, including the default timeout
	 *
	 * @param trajectory
	 */
	public static Command cmdWithAccuracy(AutoTrajectory trajectory, Distance epsilonDist) {
		return cmdWithAccuracy(trajectory, AutoConstants.kDefaultTrajectoryTimeout, epsilonDist);
	}

	public static Command cmdWithAccuracy(AutoTrajectory trajectory) {
		return cmdWithAccuracy(trajectory, AutoConstants.kAutoLinearEpsilon);
	}

	public static Command cmdWithInterrupt(AutoTrajectory trajectory) {
		return trajectory.cmd().handleInterrupt(() -> {
			Drive.mInstance.setSwerveRequest(new SwerveRequest.ApplyRobotSpeeds());
			SmartDashboard.putNumber("Auto Trajectory Command Interrupted", Timer.getFPGATimestamp());
		});
	}

	private static boolean rotationIsFinished(AutoTrajectory trajectory) {
		Pose2d currentPose = Drive.mInstance.getPose();
		Pose2d finalPose = trajectory.getFinalPose().get();
		Angle epsilonAngle = AutoConstants.kAutoAngleEpsilon;

		return MathUtil.angleModulus(Math.abs(
						currentPose.getRotation().minus(finalPose.getRotation()).getRadians()))
				< epsilonAngle.in(Units.Radians);
	}

	private static boolean translationIsFinished(AutoTrajectory trajectory, Distance epsilonDist) {
		Pose2d currentPose = Drive.mInstance.getPose();
		Pose2d finalPose = trajectory.getFinalPose().get();

		SmartDashboard.putNumber(
				"Choreo/Distance Away Inches",
				currentPose.getTranslation().getDistance(finalPose.getTranslation()) * 39.37);

		return currentPose.getTranslation().getDistance(finalPose.getTranslation()) < epsilonDist.in(Units.Meters);
	}

	private static boolean isFinished(AutoTrajectory trajectory, Distance epsilonDist) {
		boolean translationCompleted = translationIsFinished(trajectory, epsilonDist);
		boolean rotationCompleted = rotationIsFinished(trajectory);

		SmartDashboard.putBoolean("Choreo/Translation Completed", translationCompleted);
		SmartDashboard.putBoolean("Choreo/Rotation Completed", rotationCompleted);

		if (translationCompleted && rotationCompleted) {
			stopwatch.startIfNotRunning();
			if (stopwatch.getTime().gte(AutoConstants.kDelayTime)) {
				stopwatch.reset();
				return true;
			}
		} else if (!translationCompleted || !rotationCompleted) {
			stopwatch.reset();
		}

		SmartDashboard.putNumber("Choreo/Stopwatch Time", stopwatch.getTimeAsDouble());
		return false;
	}

	/**
	 * Runs a path until a coral is detected in the End Effector, intended to 'sweep' at the coral station
	 *
	 * @param trajectory
	 */
	public Command sweep(AutoTrajectory trajectory) {
		return cmdWithAccuracy(trajectory).until(() -> Superstructure.endEffectorCoralBreak.getDebouncedIfReal());
	}

	/**
	 * Runs an auto score command (equivalent to tele) given that the elevator is already raised
	 *
	 * @param wantedBranch
	 */
	public static Command autoScoreGivenPrep(Branch wantedBranch, Level wantedLevel) {
		return Commands.parallel(
						Commands.sequence(
								Superstructure.mInstance.waitUntilDriveReadyToScoreAndStable(),
								Superstructure.mInstance.coralScore(Level.L4).asProxy()),
						AutoHelpers.getAutoScorePathFromDrivePose(wantedBranch, wantedLevel))
				.withDeadline(Commands.waitUntil(() -> Superstructure.mInstance.getSuperstructureDone()))
				.withName("Autonomous L4 Score");
	}

	/**
	 * Runs an auto score command (equivalent to tele) including the elevator raise
	 *
	 * @param wantedBranch
	 */
	public static Command autoScoreWithPrep(Branch wantedBranch, Level wantedLevel) {
		return Commands.parallel(
						Commands.sequence(
								Commands.deadline(
										Superstructure.mInstance.waitToStartScoreSequence(),
										Superstructure.mInstance
												.stowCoralHold()
												.asProxy()
												.alongWith(Superstructure.mInstance
														.exhaustCoralIntake()
														.asProxy())
												.beforeStarting(
														Superstructure.endEffectorCoralBreak.stateWaitWithDebounce(
																true))),
								AutoHelpers.getPrepAndScoreForLevel(wantedLevel)
										.alongWith(CoralDeploy.mInstance
												.setpointCommand(CoralDeploy.DEPLOY)
												.asProxy())),
						AutoHelpers.getAutoScorePathFromDrivePose(wantedBranch, wantedLevel))
				.withDeadline(Commands.waitUntil(() -> Superstructure.mInstance.getSuperstructureDone()))
				.withName("Autonomous L4 Score Without Choreo");
	}

	/**
	 * Runs an auto score command (equivalent to tele) including the elevator raise
	 *
	 * @param wantedBranch
	 */
	public static Command autoScoreWithPrepWithoutCoralHold(Branch wantedBranch, Level wantedLevel) {
		return Commands.parallel(
						Commands.sequence(
								Superstructure.mInstance.waitToStartScoreSequence(),
								AutoHelpers.getPrepAndScoreForLevel(wantedLevel)),
						AutoHelpers.getAutoScorePathFromDrivePose(wantedBranch, wantedLevel))
				.withDeadline(Commands.waitUntil(() -> Superstructure.mInstance.getSuperstructureDone()))
				.withName("Autonomous L4 Score Without Choreo");
	}

	/**
	 * Runs an auto score command (equivalent to tele) including the elevator raise
	 *
	 * @param wantedBranch
	 */
	public static Command autoScoreWithPrepWithoutCoralHold(Branch wantedBranch, Level wantedLevel, Pose2d startPose) {
		return Commands.parallel(
						Commands.sequence(
								Superstructure.mInstance.waitToStartScoreSequence(),
								AutoHelpers.getPrepAndScoreForLevel(wantedLevel)),
						AutoHelpers.getAutoScorePathWithStartPose(wantedBranch, wantedLevel, startPose))
				.withDeadline(Commands.waitUntil(() -> Superstructure.mInstance.getSuperstructureDone()))
				.withName("Autonomous L4 Score Without Choreo");
	}

	/**
	 * Runs an auto score command from a trajectory including the elevator raise
	 *
	 * @param wantedBranch
	 */
	public static Command autoScoreWithPrepFromTraj(Branch wantedBranch, Level wantedLevel) {
		return Commands.parallel(
						Commands.sequence(
								Commands.deadline(
										Superstructure.mInstance.waitToStartScoreSequence(),
										Superstructure.mInstance
												.stowCoralHold()
												.asProxy()
												.alongWith(Superstructure.mInstance
														.exhaustCoralIntake()
														.asProxy())
												.beforeStarting(
														Superstructure.endEffectorCoralBreak.stateWaitWithDebounce(
																true))),
								AutoHelpers.getPrepAndScoreForLevel(wantedLevel)
										.alongWith(CoralDeploy.mInstance
												.setpointCommand(CoralDeploy.DEPLOY)
												.asProxy())),
						AutoHelpers.getAutoScoreTrajectoryFromDrivePose(wantedBranch, wantedLevel))
				.withDeadline(Commands.waitUntil(() -> Superstructure.mInstance.getSuperstructureDone()))
				.withName("Autonomous L4 Score Without Choreo");
	}

	public static Command intakeAndScoreGroundCoral(
			String trajName, Branch wantedBranch, Level wantedLevel, boolean useTraj) {
		Superstructure s = Superstructure.mInstance;

		AutoTrajectory start = routine.trajectory(trajName, 0);
		AutoTrajectory intake = routine.trajectory(trajName, 1);

		return Commands.sequence(
				Commands.deadline(start.cmd(), s.tuck().asProxy()),
				Commands.deadline(
						s.coralIntakeToEndEffector().asProxy(),
						new DetectionPIDToPoseCommand(intake, side)
								.andThen(Commands.either(
												AutoHelpers.getAutoScoreTrajectoryFromDrivePose(
														wantedBranch, wantedLevel),
												AutoHelpers.getAutoScorePathFromDrivePose(wantedBranch, wantedLevel),
												() -> useTraj)
										.until(() -> Detection.mInstance.hasCoral())
										.withTimeout(Units.Seconds.of(0.8)))
								.repeatedly()),
				Commands.either(
						autoScoreWithPrepFromTraj(wantedBranch, wantedLevel),
						autoScoreWithPrep(wantedBranch, wantedLevel),
						() -> useTraj));
	}

	public static Command intakeAndScoreGroundCoral(String trajName, Branch wantedBranch, Level wantedLevel) {
		return intakeAndScoreGroundCoral(trajName, wantedBranch, wantedLevel, false);
	}

	public static Command intakeAndScoreMark(
			String intakeTrajName, Branch wantedBranch, Level wantedLevel, boolean useTraj) {
		Superstructure s = Superstructure.mInstance;

		AutoTrajectory mark = routine.trajectory(intakeTrajName);

		return Commands.sequence(
				Commands.deadline(
						s.coralIntakeToEndEffector().asProxy(),
						mark.cmd()
								.andThen(new PIDToPoseCommand(
												mark.getFinalPose()
														.get()
														.transformBy(new Transform2d(
																Translation2d.kZero, Rotation2d.fromDegrees(20))),
												AutoConstants.kAutoLinearEpsilon.times(3.0),
												AutoConstants.kAutoAngleEpsilon.times(3.0),
												AutoConstants.getMarkTranslationController(),
												AutoConstants.getMarkHeadingController())
										.until(() -> Superstructure.indexerBreak.getDebounced())
										.andThen(Commands.either(
														AutoHelpers.getAutoScoreTrajectoryFromDrivePose(
																wantedBranch, wantedLevel),
														AutoHelpers.getAutoScorePathFromDrivePose(
																wantedBranch, wantedLevel),
														() -> useTraj)
												.until(() -> Detection.mInstance.hasCoral())
												.withTimeout(Units.Seconds.of(1.0))
												.andThen(new DetectionPIDToPoseCommand(mark, AutoType.MARK))
												.repeatedly()))),
				Commands.either(
						autoScoreWithPrepFromTraj(wantedBranch, wantedLevel),
						autoScoreWithPrep(wantedBranch, wantedLevel),
						() -> useTraj));
	}

	public static Command intakeAndScoreMarkWithChoreo(
			String intakeTrajName, String scoreTrajName, Branch wantedBranch, Level wantedLevel, boolean useTraj) {
		Superstructure s = Superstructure.mInstance;

		AutoTrajectory mark = routine.trajectory(intakeTrajName);
		AutoTrajectory score = routine.trajectory(scoreTrajName);

		return Commands.sequence(
				Commands.deadline(
						s.coralIntakeToEndEffector().asProxy(),
						mark.cmd()
								.andThen(new PIDToPoseCommand(
												mark.getFinalPose()
														.get()
														.transformBy(new Transform2d(
																Translation2d.kZero, Rotation2d.fromDegrees(20))),
												AutoConstants.kAutoLinearEpsilon.times(3.0),
												AutoConstants.kAutoAngleEpsilon.times(3.0),
												AutoConstants.getMarkTranslationController(),
												AutoConstants.getMarkHeadingController())
										.until(() -> Superstructure.indexerBreak.getDebounced())
										.andThen(Commands.either(
														AutoHelpers.getAutoScoreTrajectoryFromDrivePose(
																wantedBranch, wantedLevel),
														AutoHelpers.getAutoScorePathFromDrivePose(
																wantedBranch, wantedLevel),
														() -> useTraj)
												.until(() -> Detection.mInstance.hasCoral())
												.withTimeout(Units.Seconds.of(1.0))
												.andThen(new DetectionPIDToPoseCommand(mark, AutoType.MARK))
												.repeatedly()))),
				score.cmd()
						.andThen(Commands.either(
								autoScoreWithPrepFromTraj(wantedBranch, wantedLevel),
								autoScoreWithPrep(wantedBranch, wantedLevel),
								() -> useTraj)));
	}

	public Command netAutoScoreWithPrep(Pose2d pose, boolean isL3) {
		return Commands.parallel(
						Commands.sequence(
								Commands.parallel(
												Superstructure.mInstance.setHasAlgaeCommand(true),
												EndEffector.mInstance
														.setpointCommand(EndEffector.ALGAE_HOLD)
														.beforeStarting(Commands.waitSeconds(0.2)))
										.asProxy(),
								Superstructure.mInstance.waitUnitlSlowEnoughToRaiseNetInAuto(),
								Superstructure.mInstance.netPrep().asProxy()),
						new PIDToPoseCommand(
										pose,
										SuperstructureConstants.kNetScoringDistanceEpsilon,
										SuperstructureConstants.kNetScoringAngleEpsilon,
										DriveConstants.mAutoAlignTippyTranslationController,
										DriveConstants.mAutoAlignTippyHeadingController)
								.beforeStarting(() -> Superstructure.mInstance.setDriveReady(false))
								.until(() -> Superstructure.mInstance.getDriveReady()))
				.andThen(Superstructure.mInstance.netScore().asProxy())
				.withName("Autonomous L4 Score Without Choreo");
	}

	public void prepRoutine(Command... sequence) {
		routine.active()
				.onTrue(Commands.sequence(sequence)
						.alongWith(EndEffector.mInstance
								.setpointCommand(EndEffector.CORAL_HOLD)
								.asProxy())
						.alongWith(Commands.runOnce(() -> Elevator.mInstance.setCurrentPosition(
								ElevatorConstants.converter.toAngle(ElevatorConstants.kStowPosition))))
						.withName("Auto Routine Sequential Command Group"));
	}

	public void atTranslation(String eventName, Command event, Distance epsilon, AutoTrajectory... trajectories) {
		for (AutoTrajectory trajectory : trajectories) {
			trajectory.atTranslation(eventName, epsilon.in(Units.Meters)).onTrue(event);
		}
	}

	public void atTranslation(String eventName, Command event, AutoTrajectory... trajectories) {
		atTranslation(eventName, event, AutoConstants.kAutoLinearEpsilon, trajectories);
	}

	public void logTrajectories(AutoTrajectory... trajectories) {
		List<AutoTrajectory> list = Arrays.asList(trajectories);
		for (int i = 1; i <= list.size(); ++i) {
			if (RobotConstants.isRedAlliance) {
				LogUtil.recordTrajectory(
						"Autos/Choreo Path " + i,
						list.get(i - 1).getRawTrajectory().flipped());
			} else {
				LogUtil.recordTrajectory(
						"Autos/Choreo Path " + i, list.get(i - 1).getRawTrajectory());
			}
		}
	}

	public AutoRoutine getRoutine() {
		return routine;
	}

	public Command asCommand() {
		return routine.cmd();
	}
}
