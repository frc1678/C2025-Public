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

public class DetectionFCDE extends AutoModeBase {
	public DetectionFCDE(AutoFactory factory, AutoEndBehavior endBehavior) {
		super(factory, "Detection FCDE", AutoType.RIGHT);

		Superstructure s = Superstructure.mInstance;
		Detection d = Detection.mInstance;

		AutoTrajectory leftStartToF = trajectory("leftStartToF");

		// for coral-related ending action (no longer used)
		AutoTrajectory eToDetectionStart = trajectory("eToDetection", 0);
		AutoTrajectory eToDetectionIntake = trajectory("eToDetection", 1);

		// for ground-algae-related ending action
		AutoTrajectory eToGroundAlgae = trajectory("eToGroundAlgae");
		AutoTrajectory efAlgaeToHold = trajectory("efAlgaeToHold");

		prepRoutine(
				AutoHelpers.resetPoseIfWithoutEstimate(
						leftStartToF.getInitialPose().get()),
				leftStartToF.cmd().andThen(autoScoreWithPrepWithoutCoralHold(Branch.F, Level.L4)),
				intakeAndScoreGroundCoral("fToDetection", Branch.C, Level.L4),
				intakeAndScoreGroundCoral("cToDetection", Branch.D, Level.L4),
				intakeAndScoreGroundCoral("dToDetection", Branch.E, Level.L4, true),
				Commands.either(
						Commands.deadline(
								cmdWithAccuracy(eToGroundAlgae), s.tuck().asProxy()),
						Commands.sequence(
								s.reefAlgaeIntakeInAuto(true)
										.asProxy()
										.deadlineFor(s.goToReefIntakeReadyPose(Face.FAR_RIGHT)
												.andThen(s.goToReefIntakePose(Face.FAR_RIGHT))),
								cmdWithAccuracy(efAlgaeToHold)
										.alongWith(s.stowAlgaeWhenReady().asProxy())),
						() -> endBehavior == AutoEndBehavior.ALGAE_DRIVE));
	}
}
