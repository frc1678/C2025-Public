package frc.lib.drive;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.units.BaseUnits;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.lib.util.DelayedBoolean;
import frc.lib.util.FieldLayout;
import frc.lib.util.FieldLayout.Level;
import frc.robot.RobotConstants;
import frc.robot.controlboard.ControlBoard;
import frc.robot.controlboard.ControlBoardConstants;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.DriveConstants;
import frc.robot.subsystems.superstructure.Superstructure;
import frc.robot.subsystems.superstructure.SuperstructureConstants;
import frc.robot.subsystems.vision.Limelight;

public class FollowXLineSwerveRequestCommand extends Command {
	Distance desiredX = Units.Meters.of(0.0);
	Rotation2d targetRotation = new Rotation2d();
	LinearVelocity speedDesired = Units.MetersPerSecond.of(0.0);
	DelayedBoolean atTarget;
	Distance epsilonDist = SuperstructureConstants.getAutoAlignScoringDistanceEpsilon(Level.NET);
	Angle epsilonAngle = SuperstructureConstants.getAutoAlignScoringAngleEpsilon(Level.NET);

	/**
	 * No parameters in current implementation; hard-coded to target the net
	 */
	public FollowXLineSwerveRequestCommand() {
		addRequirements(Drive.mInstance);
		atTarget = new DelayedBoolean(
				Timer.getFPGATimestamp(),
				SuperstructureConstants.getAutoAlignScoringDelay(Level.NET).in(Units.Seconds));
	}

	public boolean driveDone() {
		return atEndPose() && !hasRecentEstimate();
	}

	@Override
	public void initialize() {
		Distance driveX = Drive.mInstance.getPose().getMeasureX();
		boolean useRedAlliance = driveX.minus(FieldLayout.getNetScoreLine(true)).abs(BaseUnits.DistanceUnit)
				< driveX.minus(FieldLayout.getNetScoreLine(false)).abs(BaseUnits.DistanceUnit);

		desiredX = FieldLayout.getNetScoreLine(useRedAlliance);
		targetRotation = getAngle(useRedAlliance);
		Superstructure.mInstance.setDriveReady(false);
		Superstructure.mInstance.setSuperstructureDone(false);
	}

	@Override
	public void execute() {
		ChassisSpeeds desiredSpeeds = calculateSpeeds();
		Drive.mInstance.setSwerveRequest(DriveConstants.PIDToPoseRequest.withVelocityX(desiredSpeeds.vxMetersPerSecond)
				.withVelocityY(desiredSpeeds.vyMetersPerSecond)
				.withRotationalRate(desiredSpeeds.omegaRadiansPerSecond));

		if (driveDone()) {
			Superstructure.mInstance.setDriveReady(true);
			ControlBoard.mInstance.setRumble(true);
		} else {
			Superstructure.mInstance.setDriveReady(false);
			ControlBoard.mInstance.setRumble(false);
		}
	}

	public ChassisSpeeds calculateSpeeds() {
		LinearVelocity ySpeedDesired = DriveConstants.kMaxSpeedTippy.times(
				DriveConstants.getDeadbandedStick(ControlBoardConstants.mDriverController.getLeftX() * -1.0));

		Pose2d currentPose = Drive.mInstance.getPose();
		Distance currentX = currentPose.getMeasureX();

		LinearVelocity xSpeedDesired = Units.MetersPerSecond.of(
						DriveConstants.mStayOnLineTranslationController.calculate(
								desiredX.minus(currentX).in(Units.Meters)))
				.times(RobotConstants.isRedAlliance ? 1.0 : -1.0);

		LinearVelocity speedDesired = Units.MetersPerSecond.of(
				Math.hypot(xSpeedDesired.in(Units.MetersPerSecond), ySpeedDesired.in(Units.MetersPerSecond)));

		Rotation2d directionToTravel =
				new Rotation2d(xSpeedDesired.in(Units.MetersPerSecond), ySpeedDesired.in(Units.MetersPerSecond));

		return new ChassisSpeeds(
				speedDesired.times(directionToTravel.getCos()),
				speedDesired.times(directionToTravel.getSin()),
				Units.RotationsPerSecond.of(DriveConstants.mStayOnLineHeadingController.calculate(
						currentPose.getRotation().minus(targetRotation).getRotations())));
	}

	public boolean atEndPose() {
		Pose2d currentPose = Drive.mInstance.getPose();
		Distance currentX = currentPose.getMeasureX();

		boolean translationComplete = currentX.minus(desiredX).lte(epsilonDist);
		boolean rotationComplete = MathUtil.angleModulus(
						Math.abs(currentPose.getRotation().minus(targetRotation).getRadians()))
				< epsilonAngle.in(Units.Radians);

		boolean complete = atTarget.update(Timer.getFPGATimestamp(), translationComplete && rotationComplete);

		SmartDashboard.putBoolean("Net Align/Completed", complete);
		SmartDashboard.putBoolean("Net Align/Translation Completed", translationComplete);
		SmartDashboard.putBoolean("Net Align/Rotation Completed", rotationComplete);
		SmartDashboard.putNumber(
				"Net Align/Distance Away Inches", currentX.minus(desiredX).abs(BaseUnits.DistanceUnit) * 39.37);

		return complete;
	}

	private Rotation2d getAngle(boolean useRedSideLine) {
		if (RobotConstants.isRedAlliance) {
			if (useRedSideLine) {
				return FieldLayout.getNetScoreAngle(useRedSideLine);
			} else {
				return FieldLayout.getNetScoreAngle(useRedSideLine).unaryMinus();
			}
		} else {
			if (useRedSideLine) {
				return FieldLayout.getNetScoreAngle(useRedSideLine).unaryMinus();
			} else {
				return FieldLayout.getNetScoreAngle(useRedSideLine);
			}
		}
	}

	@Override
	public boolean isFinished() {
		return Superstructure.mInstance.getSuperstructureDone();
	}

	public boolean hasRecentEstimate() {
		return Units.Seconds.of(Timer.getFPGATimestamp())
				.minus(Limelight.mInstance.getLastUpdateTime())
				.lte(SuperstructureConstants.kRecentUpdateTime);
	}

	@Override
	public void end(boolean interrupted) {
		ControlBoard.mInstance.setRumble(false);
		super.end(interrupted);
	}
}
