package frc.robot.autos.detection;

import choreo.auto.AutoFactory;
import choreo.auto.AutoTrajectory;
import frc.lib.util.FieldLayout.Branch;
import frc.lib.util.FieldLayout.Branch.Face;
import frc.lib.util.FieldLayout.Level;
import frc.robot.autos.AutoModeBase;
import frc.robot.subsystems.detection.Detection;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.pivot.Pivot;
import frc.robot.subsystems.superstructure.Superstructure;

public class Detection2BA extends AutoModeBase {
	public Detection2BA(AutoFactory factory) {
		super(factory, "Detection 2BA");

		Superstructure s = Superstructure.mInstance;
		Detection d = Detection.mInstance;

		AutoTrajectory rightStartToB = trajectory("rightStartToB");
		AutoTrajectory abAlgaeToHold = trajectory("abAlgaeToHold");

		prepRoutine(
				Drive.mInstance
						.resetPoseCmd(rightStartToB.getInitialPose().get())
						.alongWith(Pivot.mInstance
								.setpointCommand(Pivot.CORAL_HOLD)
								.asProxy()),
				rightStartToB.cmd().andThen(autoScoreWithPrepWithoutCoralHold(Branch.B, Level.L4)),
				intakeAndScoreMark("bToMidMark", Branch.A, Level.L4, false),
				intakeAndScoreMark("aToLeftMark", Branch.A, Level.L2, true),
				intakeAndScoreMark("aToRightMark", Branch.B, Level.L2, true),
				s.reefAlgaeIntakeInAuto(true)
						.asProxy()
						.deadlineFor(s.goToReefIntakeReadyPose(Face.NEAR_CENTER)
								.andThen(s.goToReefIntakePose(Face.NEAR_CENTER))),
				cmdWithAccuracy(abAlgaeToHold).alongWith(s.stowAlgaeWhenReady().asProxy()));
	}
}
