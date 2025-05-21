package frc.robot.autos.single;

import choreo.auto.AutoFactory;
import choreo.auto.AutoTrajectory;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.lib.util.FieldLayout;
import frc.lib.util.FieldLayout.Branch;
import frc.lib.util.FieldLayout.Level;
import frc.robot.RobotConstants;
import frc.robot.autos.AutoHelpers;
import frc.robot.autos.AutoModeBase;

public class BranchIAuto extends AutoModeBase {

	public BranchIAuto(AutoFactory factory) {
		super(factory, "Branch I Auto");

		AutoTrajectory iToStation = trajectory("iToStation");

		Pose2d startPose =
				FieldLayout.handleAllianceFlip(new Pose2d(7.17, 6.13, Rotation2d.kZero), RobotConstants.isRedAlliance);

		prepRoutine(
				AutoHelpers.resetPoseIfWithoutEstimate(startPose),
				Commands.waitSeconds(6.0),
				autoScoreWithPrepWithoutCoralHold(Branch.I, Level.L4, startPose),
				cmdWithAccuracy(iToStation));
	}
}
