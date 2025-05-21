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

public class HumanJKL extends AutoModeBase {
	public HumanJKL(AutoFactory factory) {
		super(factory, "Human From Choreo");

		Superstructure s = Superstructure.mInstance;

		AutoTrajectory midStartToJ = trajectory("midStartToJ");
		AutoTrajectory jToHumanLoad = trajectory("jToHumanLoad");
		AutoTrajectory humanLoadToK = trajectory("humanLoadToK");
		AutoTrajectory kToHumanLoad = trajectory("kToHumanLoad");
		AutoTrajectory humanLoadToL = trajectory("humanLoadToL");
		AutoTrajectory lToTuck = trajectory("lToTuck");

		prepRoutine(
				AutoHelpers.resetPoseIfWithoutEstimate(
						midStartToJ.getInitialPose().get()),
				midStartToJ
						.cmd()
						.alongWith(s.liberateCoralDeploy().asProxy())
						.andThen(autoScoreWithPrepWithoutCoralHold(Branch.J, Level.L4)),
				cmdWithAccuracy(jToHumanLoad)
						.andThen(s.stationIntake()
								.asProxy()
								.alongWith(Commands.runOnce(() -> Drive.mInstance.setSwerveRequest(
										new SwerveRequest.RobotCentric().withVelocityX(-1.0))))),
				humanLoadToK
						.cmd()
						.andThen(autoScoreWithPrepWithoutCoralHold(Branch.K, Level.L4))
						.beforeStarting(humanLoadToK.resetOdometry()),
				cmdWithAccuracy(kToHumanLoad)
						.andThen(s.stationIntake()
								.asProxy()
								.alongWith(Commands.runOnce(() -> Drive.mInstance.setSwerveRequest(
										new SwerveRequest.RobotCentric().withVelocityX(-1.5))))),
				humanLoadToL
						.cmd()
						.andThen(autoScoreWithPrepWithoutCoralHold(Branch.L, Level.L4))
						.beforeStarting(humanLoadToL.resetOdometry()),
				cmdWithAccuracy(lToTuck));
	}
}
