package frc.robot.autos;

import choreo.auto.AutoChooser;
import choreo.auto.AutoFactory;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.autos.AutoConstants.AutoEndBehavior;
import frc.robot.autos.detection.Detection2AB;
import frc.robot.autos.detection.Detection2BA;
import frc.robot.autos.detection.DetectionE2CD;
import frc.robot.autos.detection.DetectionEDCBA;
import frc.robot.autos.detection.DetectionFCDE;
import frc.robot.autos.detection.DetectionILKJ;
import frc.robot.autos.detection.DetectionJ2LK;
import frc.robot.autos.detection.DetectionJKLAB;
import frc.robot.autos.empty.Empty;
import frc.robot.autos.human.HumanEDC;
import frc.robot.autos.human.HumanJKL;
import frc.robot.autos.net.NetGHEFIJ;
import frc.robot.autos.net.NetGHIJEF;
import frc.robot.autos.net.NetGHIJKL;
import frc.robot.autos.single.BranchFAuto;
import frc.robot.autos.single.BranchHAuto;
import frc.robot.autos.single.BranchIAuto;

public class AutoModeSelector {
	private AutoChooser mAutoChooser = new AutoChooser();

	public AutoModeSelector(AutoFactory factory) {
		mAutoChooser.addRoutine("[CENTER] Net GH IJ KL", () -> new NetGHIJKL(factory).getRoutine());
		mAutoChooser.addRoutine("[CENTER] Net GH IJ EF", () -> new NetGHIJEF(factory).getRoutine());
		mAutoChooser.addRoutine("[CENTER] Net GH EF IJ", () -> new NetGHEFIJ(factory).getRoutine());

		mAutoChooser.addRoutine(
				"[RIGHT] Detection FCDE Hold Algae",
				() -> new DetectionFCDE(factory, AutoEndBehavior.ALGAE_GRAB).getRoutine());
		mAutoChooser.addRoutine(
				"[RIGHT] Detection FCDE Ground Algae Drive",
				() -> new DetectionFCDE(factory, AutoEndBehavior.ALGAE_DRIVE).getRoutine());

		mAutoChooser.addRoutine(
				"[LEFT] Detection ILKJ Hold Algae",
				() -> new DetectionILKJ(factory, AutoEndBehavior.ALGAE_GRAB).getRoutine());
		mAutoChooser.addRoutine(
				"[LEFT] Detection ILKJ Ground Algae Drive",
				() -> new DetectionILKJ(factory, AutoEndBehavior.ALGAE_DRIVE).getRoutine());

		mAutoChooser.addRoutine("[ARCHIVE, RIGHT] Human EDC", () -> new HumanEDC(factory).getRoutine());
		mAutoChooser.addRoutine("[ARCHIVE, LEFT] Human JKL", () -> new HumanJKL(factory).getRoutine());

		mAutoChooser.addRoutine("[ARCHIVE, LEFT] Detection 2AB", () -> new Detection2AB(factory).getRoutine());
		mAutoChooser.addRoutine("[ARCHIVE, RIGHT] Detection 2BA", () -> new Detection2BA(factory).getRoutine());

		mAutoChooser.addRoutine("[ARCHIVE, RIGHT] Detection E2CD", () -> new DetectionE2CD(factory).getRoutine());
		mAutoChooser.addRoutine("[ARCHIVE, RIGHT] Detection EDCBA", () -> new DetectionEDCBA(factory).getRoutine());

		mAutoChooser.addRoutine("[ARCHIVE, LEFT] Detection J2LK", () -> new DetectionJ2LK(factory).getRoutine());
		mAutoChooser.addRoutine("[ARCHIVE, LEFT] Detection JKLAB", () -> new DetectionJKLAB(factory).getRoutine());

		mAutoChooser.addRoutine("1 H Auto", () -> new BranchHAuto(factory).getRoutine());
		mAutoChooser.addRoutine("1 F Auto", () -> new BranchFAuto(factory).getRoutine());
		mAutoChooser.addRoutine("1 I Auto", () -> new BranchIAuto(factory).getRoutine());

		mAutoChooser.addRoutine("Empty", () -> new Empty(factory).getRoutine());
		SmartDashboard.putData(mAutoChooser);
	}

	public Command getSelectedCommand() {
		return mAutoChooser.selectedCommandScheduler();
	}

	public AutoChooser getAutoChooser() {
		return mAutoChooser;
	}
}
