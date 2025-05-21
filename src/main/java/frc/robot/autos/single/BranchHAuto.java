package frc.robot.autos.single;

import choreo.auto.AutoFactory;
import choreo.auto.AutoTrajectory;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.lib.util.FieldLayout;
import frc.lib.util.FieldLayout.Branch;
import frc.lib.util.FieldLayout.Level;
import frc.robot.RobotConstants;
import frc.robot.autos.AutoHelpers;
import frc.robot.autos.AutoModeBase;

public class BranchHAuto extends AutoModeBase {

	public BranchHAuto(AutoFactory factory) {
		super(factory, "BranchHAuto");
		AutoTrajectory hToStation = trajectory("hToStation");

		Pose2d startPose = FieldLayout.handleAllianceFlip(
				new Pose2d(7.17, FieldLayout.kFieldWidth.div(2.0).in(Units.Meters), Rotation2d.kZero),
				RobotConstants.isRedAlliance);

		prepRoutine(
				AutoHelpers.resetPoseIfWithoutEstimate(startPose),
				Commands.waitSeconds(6.0),
				autoScoreWithPrepWithoutCoralHold(Branch.H, Level.L4, startPose),
				cmdWithAccuracy(hToStation));
	}
}
