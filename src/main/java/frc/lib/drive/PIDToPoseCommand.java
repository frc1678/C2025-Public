package frc.lib.drive;

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
import frc.lib.logging.LogUtil;
import frc.lib.util.DelayedBoolean;
import frc.lib.util.FieldLayout.Level;
import frc.lib.util.SynchronousPIDF;
import frc.lib.util.Util;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.DriveConstants;
import frc.robot.subsystems.superstructure.Superstructure;
import frc.robot.subsystems.superstructure.SuperstructureConstants;

public class PIDToPoseCommand extends Command {
	Pose2d finalPose;
	Rotation2d targetRotation;
	Distance epsilonDist;
	Angle epsilonAngle;
	Util.Pose2dTimeInterpolable interpolable;
	DelayedBoolean atTarget;
	boolean isAuto;
	SynchronousPIDF translationController;
	SynchronousPIDF headingController;

	public PIDToPoseCommand(
			Pose2d finalPose,
			Distance epsilonDist,
			Angle epsilonAngle,
			Time delayTime,
			Rotation2d targetRotation,
			SynchronousPIDF translationController,
			SynchronousPIDF headingController) {
		addRequirements(Drive.mInstance);

		this.finalPose = finalPose;
		this.epsilonDist = epsilonDist;
		this.epsilonAngle = epsilonAngle;
		this.targetRotation = targetRotation.plus(Rotation2d.k180deg);
		this.translationController = translationController;
		this.headingController = headingController;

		atTarget = new DelayedBoolean(Timer.getFPGATimestamp(), delayTime.in(Units.Seconds));
	}

	public PIDToPoseCommand(
			Pose2d finalPose, Distance epsilonDist, Angle epsilonAngle, Time delayTime, Rotation2d targetRotation) {
		this(
				finalPose,
				epsilonDist,
				epsilonAngle,
				delayTime,
				targetRotation,
				DriveConstants.mAutoAlignTranslationController,
				DriveConstants.mAutoAlignHeadingController);
	}

	public PIDToPoseCommand(
			Pose2d finalPose,
			Distance epsilonDist,
			Angle epsilonAngle,
			Time delayTime,
			SynchronousPIDF translationController,
			SynchronousPIDF headingController) {
		this(
				finalPose,
				epsilonDist,
				epsilonAngle,
				delayTime,
				finalPose.getRotation(),
				translationController,
				headingController);
	}

	public PIDToPoseCommand(
			Pose2d finalPose,
			Time delayTime,
			Rotation2d targetRotation,
			SynchronousPIDF translationController,
			SynchronousPIDF headingController) {
		this(
				finalPose,
				Units.Centimeters.of(4.0),
				Units.Degrees.of(0.8),
				delayTime,
				targetRotation,
				translationController,
				headingController);
	}

	public PIDToPoseCommand(
			Pose2d finalPose,
			Time delayTime,
			SynchronousPIDF translationController,
			SynchronousPIDF headingController) {
		this(finalPose, delayTime, finalPose.getRotation(), translationController, headingController);
	}

	/**
	 * Boolean param was added to avoid confusion with the constructor that would transform a scoring pose from a branch.
	 * For example, this is used for our auto align to L1.
	 */
	public PIDToPoseCommand(Pose2d rawEndPose, Level level, boolean useRaw) {
		this(
				rawEndPose.transformBy(new Transform2d(new Translation2d(), Rotation2d.k180deg)),
				SuperstructureConstants.getAutoAlignScoringDistanceEpsilon(level),
				SuperstructureConstants.getAutoAlignScoringAngleEpsilon(level),
				SuperstructureConstants.getAutoAlignScoringDelay(level),
				rawEndPose.getRotation(),
				DriveConstants.mAutoAlignTranslationController,
				DriveConstants.mAutoAlignHeadingController);
	}

	public PIDToPoseCommand(
			Pose2d finalPose,
			Distance epsilonDist,
			Angle epsilonAngle,
			SynchronousPIDF translationController,
			SynchronousPIDF headingController) {
		this(
				finalPose,
				epsilonDist,
				epsilonAngle,
				Units.Seconds.of(0.0),
				finalPose.getRotation(),
				translationController,
				headingController);
	}

	public PIDToPoseCommand(
			Pose2d finalPose, SynchronousPIDF translationController, SynchronousPIDF headingController) {
		this(finalPose, Units.Seconds.of(0.00), translationController, headingController);
	}

	public PIDToPoseCommand(Pose2d finalPose) {
		this(finalPose, DriveConstants.mAutoAlignTranslationController, DriveConstants.mAutoAlignHeadingController);
	}

	/* AUTO ALIGN PID TO POSE COMMANDS */
	public PIDToPoseCommand(
			Pose2d finalPose, Level level, SynchronousPIDF translationController, SynchronousPIDF headingController) {
		this(
				TrajectoryHelpers.getDriveTargetPose(
								finalPose, SuperstructureConstants.getAutoAlignHeadingGenerationDeadband(level), level)
						.plus(new Transform2d(Translation2d.kZero, Rotation2d.k180deg)),
				SuperstructureConstants.getAutoAlignScoringDistanceEpsilon(level),
				SuperstructureConstants.getAutoAlignScoringAngleEpsilon(level),
				SuperstructureConstants.getAutoAlignScoringDelay(level),
				finalPose.getRotation(),
				translationController,
				headingController);
	}

	public PIDToPoseCommand(Pose2d finalPose, Level level, SynchronousPIDF translationController) {
		this(finalPose, level, translationController, DriveConstants.mAutoAlignHeadingController);
	}

	public PIDToPoseCommand(Pose2d finalPose, Level level) {
		this(
				finalPose,
				level,
				DriveConstants.mAutoAlignTranslationController,
				DriveConstants.mAutoAlignHeadingController);
	}

	@Override
	public void initialize() {
		Superstructure.mInstance.setPathFollowing(true);
	}

	@Override
	public void execute() {
		LogUtil.recordPose2d("Auto Align PID/Final Pose", finalPose);
		Drive.mInstance.setSwerveRequest(
				DriveConstants.getPIDToPoseRequestUpdater(finalPose, translationController, headingController)
						.apply(DriveConstants.PIDToPoseRequest));
	}

	@Override
	public void end(boolean interrupted) {
		Superstructure.mInstance.setPathFollowing(false);
		Drive.mInstance.setSwerveRequest(new SwerveRequest.ApplyRobotSpeeds());
	}

	@Override
	public boolean isFinished() {
		return atEndPose();
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

		SmartDashboard.putBoolean("Auto Align PID/Completed", complete);
		SmartDashboard.putBoolean(
				"Auto Align PID/Translation Completed",
				currentPose.getTranslation().getDistance(finalPose.getTranslation()) < epsilonDist.in(Units.Meters));
		SmartDashboard.putBoolean(
				"Auto Align PID/Rotation Completed",
				MathUtil.angleModulus(Math.abs(currentPose
								.getRotation()
								.minus(finalPose.getRotation())
								.getRadians()))
						< epsilonAngle.in(Units.Radians));
		SmartDashboard.putNumber(
				"Auto Align PID/Distance Away Inches",
				currentPose.getTranslation().getDistance(finalPose.getTranslation()) * 39.37);

		return complete;
	}

	public Distance distanceFromEnd() {
		return Units.Meters.of(Drive.mInstance
				.getPose()
				.getTranslation()
				.minus(finalPose.getTranslation())
				.getNorm());
	}
}
