package frc.robot.autos.net;

import choreo.auto.AutoFactory;
import choreo.auto.AutoTrajectory;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.lib.drive.PIDToPoseCommand;
import frc.lib.util.FieldLayout;
import frc.lib.util.FieldLayout.Branch;
import frc.lib.util.FieldLayout.Branch.Face;
import frc.lib.util.FieldLayout.Level;
import frc.robot.Robot;
import frc.robot.RobotConstants;
import frc.robot.autos.AutoHelpers;
import frc.robot.autos.AutoModeBase;
import frc.robot.subsystems.detection.Detection;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.elevator.ElevatorConstants;
import frc.robot.subsystems.pivot.Pivot;
import frc.robot.subsystems.pivot.PivotConstants;
import frc.robot.subsystems.superstructure.MotionPlanner;
import frc.robot.subsystems.superstructure.Superstructure;

public class NetGHIJKL extends AutoModeBase {
	public NetGHIJKL(AutoFactory factory) {
		super(factory, "Net GH IJ KL");

		Superstructure s = Superstructure.mInstance;
		Detection d = Detection.mInstance;

		Pose2d startPose = FieldLayout.handleAllianceFlip(
				new Pose2d(7.17, FieldLayout.getCoralScoringPose(Branch.H).getY(), Rotation2d.kZero),
				RobotConstants.isRedAlliance);
		Pose2d firstAlignPose = FieldLayout.handleAllianceFlip(
				new Pose2d(
						FieldLayout.kFieldLength
								.div(2.0)
								.minus(Units.Inches.of(29.5))
								.in(Units.Meters),
						5.0,
						FieldLayout.netScoreAngle),
				RobotConstants.isRedAlliance);
		Pose2d secondAlignPose = FieldLayout.handleAllianceFlip(
				new Pose2d(
						FieldLayout.kFieldLength
								.div(2.0)
								.minus(Units.Inches.of(28.0))
								.in(Units.Meters),
						6.4,
						FieldLayout.netScoreAngle),
				RobotConstants.isRedAlliance);
		Pose2d thirdAlignPose = FieldLayout.handleAllianceFlip(
				new Pose2d(
						FieldLayout.kFieldLength
								.div(2.0)
								.minus(Units.Inches.of(26.0))
								.in(Units.Meters),
						6.6,
						Rotation2d.kZero),
				RobotConstants.isRedAlliance);

		Pose2d endingPose = new Pose2d(
				thirdAlignPose.getMeasureX().plus(Units.Inches.of(RobotConstants.isRedAlliance ? -4.0 : 4.0)),
				thirdAlignPose.getMeasureY(),
				thirdAlignPose.getRotation());

		AutoTrajectory ghAlgaeToNet = trajectory("ghAlgaeToNet");
		AutoTrajectory netToIJAlgae = trajectory("netToIJAlgae");
		AutoTrajectory ijAlgaeToNet = trajectory("ijAlgaeToCenterNet");
		AutoTrajectory netToKLAlgae = trajectory("netToKLAlgae");
		AutoTrajectory klAlgaeToNet = trajectory("klAlgaeToNet");

		prepRoutine(
				AutoHelpers.resetPoseIfWithoutEstimate(startPose),
				autoScoreWithPrepWithoutCoralHold(Branch.H, Level.L4),
				s.reefAlgaeIntakeInAuto(false).asProxy().deadlineFor(s.goToReefIntakePose(Face.FAR_CENTER)),
				ghAlgaeToNet
						.cmd()
						.deadlineFor(Commands.waitSeconds(0.5)
								.andThen(
										MotionPlanner.safePivotAndElevatorToPosition(Pivot.NET_SCORE, Elevator.NET_PREP)
												.unless(() -> Pivot.mInstance.nearPosition(PivotConstants.kNetScore))
												.until(() -> Pivot.mInstance.nearPosition(PivotConstants.kNetScore))
												.asProxy()))
						.andThen(netAutoScoreWithPrep(firstAlignPose, false)),
				s.reefAlgaeIntakeInAuto(true)
						.asProxy()
						.deadlineFor(netToIJAlgae.cmd().andThen(s.goToReefIntakePose(Face.FAR_LEFT))),
				ijAlgaeToNet
						.cmd()
						.deadlineFor(Commands.waitSeconds(0.5)
								.andThen(MotionPlanner.safePivotAndElevatorToPosition(Pivot.NET_SCORE, Elevator.L3_LIFT)
										.unless(() -> Pivot.mInstance.nearPosition(PivotConstants.kNetScore))
										.until(() -> Pivot.mInstance.nearPosition(PivotConstants.kNetScore))
										.asProxy()))
						.andThen(netAutoScoreWithPrep(secondAlignPose, true)),
				s.reefAlgaeIntakeInAuto(false)
						.asProxy()
						.deadlineFor(netToKLAlgae.cmd().andThen(s.goToReefIntakePose(Face.NEAR_LEFT))),
				Commands.either(
						Commands.parallel(
								Commands.waitSeconds(1.0)
										.andThen(s.processorPrep().asProxy()),
								klAlgaeToNet
										.cmd()
										.andThen(new PIDToPoseCommand(
												klAlgaeToNet.getFinalPose().get()))),
						Commands.sequence(
								klAlgaeToNet
										.cmd()
										.deadlineFor(Commands.waitSeconds(0.5)
												.andThen(MotionPlanner.safePivotAndElevatorToPosition(
																Pivot.NET_SCORE, Elevator.NET_PREP)
														.unless(() ->
																Pivot.mInstance.nearPosition(PivotConstants.kNetScore))
														.until(() ->
																Pivot.mInstance.nearPosition(PivotConstants.kNetScore))
														.asProxy())),
								netAutoScoreWithPrep(thirdAlignPose, false),
								s.tuck()
										.asProxy()
										.alongWith(
												Commands.waitUntil(() -> Elevator.mInstance
														.getPosition()
														.lte(ElevatorConstants.converter.toAngle(
																Units.Inches.of(50.0)))),
												new PIDToPoseCommand(endingPose))),
						() -> Robot.autoTimer.getTime().gte(Units.Seconds.of(15.0 - 3.42))));
	}
}
