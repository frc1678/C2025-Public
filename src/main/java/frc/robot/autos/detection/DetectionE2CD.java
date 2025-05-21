package frc.robot.autos.detection;

import choreo.auto.AutoFactory;
import choreo.auto.AutoTrajectory;
import frc.lib.util.FieldLayout.Branch;
import frc.lib.util.FieldLayout.Branch.Face;
import frc.lib.util.FieldLayout.Level;
import frc.robot.autos.AutoConstants.AutoType;
import frc.robot.autos.AutoHelpers;
import frc.robot.autos.AutoModeBase;
import frc.robot.subsystems.detection.Detection;
import frc.robot.subsystems.superstructure.Superstructure;

public class DetectionE2CD extends AutoModeBase {
	public DetectionE2CD(AutoFactory factory) {
		super(factory, "Detection E2CD", AutoType.RIGHT);

		Superstructure s = Superstructure.mInstance;
		Detection d = Detection.mInstance;

		AutoTrajectory leftStartToE = trajectory("leftStartToE");
		AutoTrajectory cdAlgaeToHold = trajectory("cdAlgaeToHold");

		prepRoutine(
				AutoHelpers.resetPoseIfWithoutEstimate(
						leftStartToE.getInitialPose().get()),
				leftStartToE.cmd().andThen(autoScoreWithPrepWithoutCoralHold(Branch.E, Level.L4)),
				intakeAndScoreGroundCoral("eToDetection", Branch.C, Level.L4),
				intakeAndScoreGroundCoral("cToDetection", Branch.D, Level.L4),
				intakeAndScoreGroundCoral("dToDetection", Branch.C, Level.L3),
				intakeAndScoreGroundCoral("cToDetection", Branch.D, Level.L3),
				s.reefAlgaeIntakeInAuto(false)
						.asProxy()
						.deadlineFor(s.goToReefIntakeReadyPose(Face.NEAR_RIGHT)
								.andThen(s.goToReefIntakePose(Face.NEAR_RIGHT))),
				cmdWithAccuracy(cdAlgaeToHold).alongWith(s.stowAlgaeWhenReady().asProxy()));
	}
}
