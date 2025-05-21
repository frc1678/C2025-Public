package frc.robot.autos.detection;

import choreo.auto.AutoFactory;
import choreo.auto.AutoTrajectory;
import frc.lib.util.FieldLayout.Branch;
import frc.lib.util.FieldLayout.Level;
import frc.robot.autos.AutoConstants.AutoType;
import frc.robot.autos.AutoHelpers;
import frc.robot.autos.AutoModeBase;
import frc.robot.subsystems.detection.Detection;
import frc.robot.subsystems.superstructure.Superstructure;

public class DetectionJKLAB extends AutoModeBase {
	public DetectionJKLAB(AutoFactory factory) {
		super(factory, "Detection JKLAB", AutoType.LEFT);

		Superstructure s = Superstructure.mInstance;
		Detection d = Detection.mInstance;

		AutoTrajectory midStartToJ = trajectory("midStartToJ");
		AutoTrajectory aToMidMark = trajectory("aToMidMark");

		prepRoutine(
				AutoHelpers.resetPoseIfWithoutEstimate(
						midStartToJ.getInitialPose().get()),
				midStartToJ.cmd().andThen(autoScoreWithPrepWithoutCoralHold(Branch.J, Level.L4)),
				intakeAndScoreGroundCoral("jToDetection", Branch.K, Level.L4),
				intakeAndScoreGroundCoral("kToDetection", Branch.L, Level.L4),
				intakeAndScoreGroundCoral("lToDetection", Branch.A, Level.L4),
				cmdWithAccuracy(aToMidMark)
						.withDeadline(s.coralIntakeToEndEffector().asProxy()),
				autoScoreWithPrep(Branch.B, Level.L4),
				s.tuck().asProxy());
	}
}
