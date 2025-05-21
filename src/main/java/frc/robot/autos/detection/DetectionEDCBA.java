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

public class DetectionEDCBA extends AutoModeBase {
	public DetectionEDCBA(AutoFactory factory) {
		super(factory, "Detection EDCB", AutoType.RIGHT);

		Superstructure s = Superstructure.mInstance;
		Detection d = Detection.mInstance;

		AutoTrajectory midStartToE = trajectory("midStartToE");
		AutoTrajectory bToMidMark = trajectory("bToMidMark");

		prepRoutine(
				AutoHelpers.resetPoseIfWithoutEstimate(
						midStartToE.getInitialPose().get()),
				midStartToE.cmd().andThen(autoScoreWithPrepWithoutCoralHold(Branch.E, Level.L4)),
				intakeAndScoreGroundCoral("eToDetection", Branch.D, Level.L4),
				intakeAndScoreGroundCoral("dToDetection", Branch.C, Level.L4),
				intakeAndScoreGroundCoral("cToDetection", Branch.B, Level.L4),
				cmdWithAccuracy(bToMidMark)
						.withDeadline(s.coralIntakeToEndEffector().asProxy()),
				autoScoreWithPrep(Branch.A, Level.L4),
				s.tuck().asProxy());
	}
}
