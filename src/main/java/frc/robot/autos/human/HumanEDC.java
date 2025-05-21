package frc.robot.autos.human;

import choreo.auto.AutoFactory;
import choreo.auto.AutoTrajectory;
import com.ctre.phoenix6.swerve.SwerveRequest;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.lib.util.FieldLayout.Branch;
import frc.lib.util.FieldLayout.Level;
import frc.robot.autos.AutoHelpers;
import frc.robot.autos.AutoModeBase;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.superstructure.Superstructure;

public class HumanEDC extends AutoModeBase {
	public HumanEDC(AutoFactory factory) {
		super(factory, "Human From Choreo");

		Superstructure s = Superstructure.mInstance;

		AutoTrajectory midStartToE = trajectory("midStartToE");
		AutoTrajectory eToHumanLoad = trajectory("eToHumanLoad");
		AutoTrajectory humanLoadToD = trajectory("humanLoadToD");
		AutoTrajectory dToHumanLoad = trajectory("dToHumanLoad");
		AutoTrajectory humanLoadToC = trajectory("humanLoadToC");
		AutoTrajectory cToTuck = trajectory("cToTuck");

		prepRoutine(
				AutoHelpers.resetPoseIfWithoutEstimate(
						midStartToE.getInitialPose().get()),
				midStartToE
						.cmd()
						.alongWith(s.liberateCoralDeploy().asProxy())
						.andThen(autoScoreWithPrepWithoutCoralHold(Branch.E, Level.L4)),
				cmdWithAccuracy(eToHumanLoad)
						.andThen(s.stationIntake()
								.asProxy()
								.alongWith(Commands.runOnce(() -> Drive.mInstance.setSwerveRequest(
										new SwerveRequest.RobotCentric().withVelocityX(-1.0))))),
				humanLoadToD
						.cmd()
						.andThen(autoScoreWithPrepWithoutCoralHold(Branch.D, Level.L4))
						.beforeStarting(humanLoadToD.resetOdometry()),
				cmdWithAccuracy(dToHumanLoad)
						.andThen(s.stationIntake()
								.asProxy()
								.alongWith(Commands.runOnce(() -> Drive.mInstance.setSwerveRequest(
										new SwerveRequest.RobotCentric().withVelocityX(-1.5))))),
				humanLoadToC
						.cmd()
						.andThen(autoScoreWithPrepWithoutCoralHold(Branch.C, Level.L4))
						.beforeStarting(humanLoadToC.resetOdometry()),
				cmdWithAccuracy(cToTuck));
	}
}
