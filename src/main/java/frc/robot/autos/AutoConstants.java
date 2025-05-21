package frc.robot.autos;

import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Time;
import frc.lib.util.SynchronousPIDF;
import frc.robot.subsystems.drive.DriveConstants;

public final class AutoConstants {
	public static final Distance kAutoLinearEpsilon = Units.Centimeters.of(4.0);
	public static final Angle kAutoAngleEpsilon = Units.Degrees.of(1.0);
	public static final Time kDelayTime = Units.Milliseconds.of(80);
	public static final Time kDefaultTrajectoryTimeout = Units.Seconds.of(1.0);
	public static final Time kDefaultCoralOuttakeTimeout = Units.Seconds.of(1.0);

	public static SynchronousPIDF getDetectionTranslationController() {
		SynchronousPIDF controller = new SynchronousPIDF(3.0, 0.0, 0.0);
		controller.setMaxAbsoluteOutput(DriveConstants.kMaxSpeed.times(0.8).in(Units.MetersPerSecond));
		return controller;
	}

	public static SynchronousPIDF getDetectionHeadingController() {
		SynchronousPIDF controller = new SynchronousPIDF(5.25, 0.0, 0.0);
		controller.setInputRange(-0.5, 0.5);
		controller.setMaxAbsoluteOutput(DriveConstants.kMaxAngularRate.in(Units.RotationsPerSecond));
		controller.setContinuous();
		return controller;
	}

	public static SynchronousPIDF getMarkTranslationController() {
		SynchronousPIDF controller = new SynchronousPIDF(2.8, 0.0, 0.0);
		controller.setMaxAbsoluteOutput(DriveConstants.kMaxSpeed.times(0.3).in(Units.MetersPerSecond));
		return controller;
	}

	public static SynchronousPIDF getMarkHeadingController() {
		SynchronousPIDF controller = new SynchronousPIDF(5.6, 0.0, 0.0);
		controller.setInputRange(-0.5, 0.5);
		controller.setMaxAbsoluteOutput(DriveConstants.kMaxAngularRate.in(Units.RotationsPerSecond));
		controller.setContinuous();
		return controller;
	}

	public static enum AutoType {
		LEFT,
		RIGHT,
		MARK
	}

	public static enum AutoEndBehavior {
		ALGAE_GRAB,
		ALGAE_DRIVE
	}
}
