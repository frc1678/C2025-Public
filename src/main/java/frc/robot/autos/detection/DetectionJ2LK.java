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

public class DetectionJ2LK extends AutoModeBase {
	public DetectionJ2LK(AutoFactory factory) {
		super(factory, "Detection J2LK", AutoType.LEFT);

		Superstructure s = Superstructure.mInstance;
		Detection d = Detection.mInstance;

		AutoTrajectory rightStartToJ = trajectory("rightStartToJ");
		AutoTrajectory klAlgaeToHold = trajectory("klAlgaeToHold");

		prepRoutine(
				AutoHelpers.resetPoseIfWithoutEstimate(
						rightStartToJ.getInitialPose().get()),
				rightStartToJ.cmd().andThen(autoScoreWithPrepWithoutCoralHold(Branch.J, Level.L4)),
				intakeAndScoreGroundCoral("jToDetection", Branch.L, Level.L4),
				intakeAndScoreGroundCoral("lToDetection", Branch.K, Level.L4),
				intakeAndScoreGroundCoral("kToDetection", Branch.L, Level.L3),
				intakeAndScoreGroundCoral("lToDetection", Branch.K, Level.L3),
				s.reefAlgaeIntakeInAuto(false)
						.asProxy()
						.deadlineFor(s.goToReefIntakeReadyPose(Face.NEAR_LEFT)
								.andThen(s.goToReefIntakePose(Face.NEAR_LEFT))),
				cmdWithAccuracy(klAlgaeToHold).alongWith(s.stowAlgaeWhenReady().asProxy()));
	}
}
