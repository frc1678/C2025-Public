package frc.robot.autos.detection;

import choreo.auto.AutoFactory;
import choreo.auto.AutoTrajectory;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.lib.util.FieldLayout.Branch;
import frc.lib.util.FieldLayout.Branch.Face;
import frc.lib.util.FieldLayout.Level;
import frc.robot.autos.AutoConstants.AutoEndBehavior;
import frc.robot.autos.AutoConstants.AutoType;
import frc.robot.autos.AutoHelpers;
import frc.robot.autos.AutoModeBase;
import frc.robot.subsystems.detection.Detection;
import frc.robot.subsystems.superstructure.Superstructure;

public class DetectionILKJ extends AutoModeBase {
	public DetectionILKJ(AutoFactory factory, AutoEndBehavior endBehavior) {
		super(factory, "Detection ILKJ", AutoType.LEFT);

		Superstructure s = Superstructure.mInstance;
		Detection d = Detection.mInstance;

		AutoTrajectory rightStartToI = trajectory("rightStartToI");

		// for coral-related ending action (no longer used)
		AutoTrajectory jToDetectionStart = trajectory("jToDetection", 0);
		AutoTrajectory jToDetectionIntake = trajectory("jToDetection", 1);

		// for ground-algae-related ending action
		AutoTrajectory jToGroundAlgae = trajectory("jToGroundAlgae");
		AutoTrajectory ijAlgaeToHold = trajectory("ijAlgaeToHold");

		prepRoutine(
				AutoHelpers.resetPoseIfWithoutEstimate(
						rightStartToI.getInitialPose().get()),
				rightStartToI.cmd().andThen(autoScoreWithPrepWithoutCoralHold(Branch.I, Level.L4)),
				intakeAndScoreGroundCoral("iToDetection", Branch.L, Level.L4),
				intakeAndScoreGroundCoral("lToDetection", Branch.K, Level.L4),
				intakeAndScoreGroundCoral("kToDetection", Branch.J, Level.L4, true),
				Commands.either(
						Commands.deadline(
								cmdWithAccuracy(jToGroundAlgae), s.tuck().asProxy()),
						Commands.sequence(
								s.reefAlgaeIntakeInAuto(true)
										.asProxy()
										.deadlineFor(s.goToReefIntakeReadyPose(Face.FAR_LEFT)
												.andThen(s.goToReefIntakePose(Face.FAR_LEFT))),
								cmdWithAccuracy(ijAlgaeToHold)
										.alongWith(s.stowAlgaeWhenReady().asProxy())),
						() -> endBehavior == AutoEndBehavior.ALGAE_DRIVE));
	}
}
