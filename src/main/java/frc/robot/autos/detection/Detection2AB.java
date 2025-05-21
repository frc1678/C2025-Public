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

public class Detection2AB extends AutoModeBase {
	public Detection2AB(AutoFactory factory) {
		super(factory, "Detection 2AB");

		Superstructure s = Superstructure.mInstance;
		Detection d = Detection.mInstance;

		AutoTrajectory leftStartToA = trajectory("leftStartToA");
		AutoTrajectory abAlgaeToHold = trajectory("abAlgaeToHold");

		prepRoutine(
				Drive.mInstance
						.resetPoseCmd(leftStartToA.getInitialPose().get())
						.alongWith(Pivot.mInstance
								.setpointCommand(Pivot.CORAL_HOLD)
								.asProxy()),
				leftStartToA.cmd().andThen(autoScoreWithPrepWithoutCoralHold(Branch.A, Level.L4)),
				intakeAndScoreMark("aToMidMark", Branch.B, Level.L4, false),
				intakeAndScoreMarkWithChoreo("bToRightMark", "rightMarkToAOrB", Branch.B, Level.L2, false),
				intakeAndScoreMarkWithChoreo("bToLeftMark", "leftMarkToAOrB", Branch.A, Level.L2, false),
				s.reefAlgaeIntakeInAuto(true)
						.asProxy()
						.deadlineFor(s.goToReefIntakeReadyPose(Face.NEAR_CENTER)
								.andThen(s.goToReefIntakePose(Face.NEAR_CENTER))),
				cmdWithAccuracy(abAlgaeToHold).alongWith(s.stowAlgaeWhenReady().asProxy()));
	}
}
