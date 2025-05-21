package frc.robot.subsystems.climber;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Angle;
import frc.lib.bases.ServoMotorSubsystem;
import frc.lib.io.MotorIO.Setpoint;
import frc.lib.io.MotorIOTalonFX;
import frc.lib.util.Util;
import java.util.ArrayList;

public class Climber extends ServoMotorSubsystem<MotorIOTalonFX> {
	private StructPublisher<Pose3d> publisher = NetworkTableInstance.getDefault()
			.getStructTopic("Mechanisms/Climber", Pose3d.struct)
			.publish();

	public static final Setpoint PULL =
			Setpoint.withMotionMagicSetpoint(ClimberConstants.converter.toAngle(ClimberConstants.kPullPosition));
	public static final Setpoint PREP =
			Setpoint.withMotionMagicSetpoint(ClimberConstants.converter.toAngle(ClimberConstants.kPrepPosition));
	public static final Setpoint STOW =
			Setpoint.withMotionMagicSetpoint(ClimberConstants.converter.toAngle(ClimberConstants.kStowPosition));
	public static final Setpoint CLEAR =
			Setpoint.withMotionMagicSetpoint(ClimberConstants.converter.toAngle(ClimberConstants.kClearPosition));

	public static final Setpoint JOG_UP = Setpoint.withVoltageSetpoint(Units.Volts.of(10.0));
	public static final Setpoint JOG_DOWN = Setpoint.withVoltageSetpoint(Units.Volts.of(-10.0));
	public static final Setpoint HOLD = Setpoint.withVoltageSetpoint(Units.Volts.of(0.0));

	public static final Climber mInstance = new Climber();

	private Climber() {
		super(
				ClimberConstants.getMotorIO(),
				"Climber",
				ClimberConstants.converter.toAngle(ClimberConstants.kEpsilonThreshold));
		setCurrentPosition(ClimberConstants.converter.toAngle(ClimberConstants.kStowPosition));
		applySetpoint(CLEAR);
	}

	@Override
	public void outputTelemetry() {
		publisher.set(ClimberConstants.kOffsetPose.plus(
				new Transform3d(new Translation3d(), new Rotation3d(getAngle().in(Units.Radians), 0.0, 0.0))));
		super.outputTelemetry();
	}

	public Angle getAngle() {
		ArrayList<Translation2d> possibleIntersections = Util.getCircleIntersectionPoints(
				ClimberConstants.kPulleyPointTranslation,
						ClimberConstants.converter.toDistance(getPosition()).in(Units.Meters),
				ClimberConstants.KPivotPointTranslation,
						ClimberConstants.kPivotPointToStringAttatchment.in(Units.Meters));
		switch (possibleIntersections.size()) {
			case 1: // 1 intersection, get angle of pivot point to intersections
				return Units.Radians.of(possibleIntersections
						.get(0)
						.minus(ClimberConstants.KPivotPointTranslation)
						.getAngle()
						.getRadians());
			case 2: // 2 intersections, correct one will be further right, get angle of pivot to that one
				Translation2d correctIntersection =
						(possibleIntersections.get(0).getX()
										> possibleIntersections.get(1).getX())
								? possibleIntersections.get(0)
								: possibleIntersections.get(1);
				return Units.Radians.of(correctIntersection
						.minus(ClimberConstants.KPivotPointTranslation)
						.getAngle()
						.getRadians());
			case 0: // No intersections between circles or ArrayList has unexpected length, something went wrong
			default:
				return Units.Degrees.of(0.0);
		}
	}
}
