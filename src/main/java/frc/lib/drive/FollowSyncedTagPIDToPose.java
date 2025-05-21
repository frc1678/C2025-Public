package frc.lib.drive;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj.Timer;
import frc.lib.util.FieldLayout.Level;
import frc.robot.Robot;
import frc.robot.controlboard.ControlBoard;
import frc.robot.subsystems.superstructure.SuperstructureConstants;
import frc.robot.subsystems.vision.Limelight;

/**
 * Adds another layer to the FollowSyncedPIDToPose command, alerting the driver when a tag is not seen while scoring so that they know that they're on their own.
 * With where our camera is placed, no matter where you're scoring on the reef you should be able to see at least one tag, so something must be up if you can't.
 */
public class FollowSyncedTagPIDToPose extends FollowSyncedPIDToPose {
	public FollowSyncedTagPIDToPose(Pose2d finalPose, Level level) {
		super(finalPose, level);
	}

	public FollowSyncedTagPIDToPose(Pose2d rawEndPose, Level level, boolean diffParam) {
		super(rawEndPose, level, diffParam);
	}

	@Override
	public boolean driveDone() {
		return super.driveDone() && (hasRecentEstimate() && latestEstimateNearTarget() || Robot.isSimulation());
	}

	@Override
	public void execute() {
		if (super.driveDone() && !hasRecentEstimate()) {
			ControlBoard.mInstance.setRumble(true);
		} else {
			ControlBoard.mInstance.setRumble(false);
		}
		super.execute();
	}

	public boolean hasRecentEstimate() {
		return Units.Seconds.of(Timer.getFPGATimestamp())
				.minus(Limelight.mInstance.getLastUpdateTime())
				.lte(SuperstructureConstants.kRecentUpdateTime);
	}

	public boolean latestEstimateNearTarget() {
		Pose2d currentVisPose = Limelight.mInstance.getLatestUpdate();

		return currentVisPose.getTranslation().getDistance(finalPose.getTranslation()) < epsilonDist.in(Units.Meters)
				&& MathUtil.angleModulus(Math.abs(currentVisPose
								.getRotation()
								.minus(finalPose.getRotation())
								.getRadians()))
						< epsilonAngle.in(Units.Radians);
	}

	@Override
	public void end(boolean interrupted) {
		ControlBoard.mInstance.setRumble(false);
		super.end(interrupted);
	}
}
