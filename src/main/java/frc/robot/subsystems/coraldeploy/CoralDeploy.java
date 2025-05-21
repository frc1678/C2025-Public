package frc.robot.subsystems.coraldeploy;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.units.BaseUnits;
import edu.wpi.first.units.measure.Angle;
import frc.lib.bases.ServoMotorSubsystem;
import frc.lib.io.MotorIO.Setpoint;
import frc.lib.io.MotorIOTalonFX;

public class CoralDeploy extends ServoMotorSubsystem<MotorIOTalonFX> {
	public static final Setpoint STOW_FULL = Setpoint.withMotionMagicSetpoint(CoralDeployConstants.kFullStowPosition);
	public static final Setpoint STOW_CLEAR = Setpoint.withMotionMagicSetpoint(CoralDeployConstants.kStowClearPosition);
	public static final Setpoint DEPLOY = Setpoint.withMotionMagicSetpoint(CoralDeployConstants.kDeployPosition);
	public static final Setpoint EXHAUST = Setpoint.withMotionMagicSetpoint(CoralDeployConstants.kExhaustPosition);
	public static final Setpoint INDEXERHOLD = Setpoint.withMotionMagicSetpoint(CoralDeployConstants.kIndexerHold);

	public static final CoralDeploy mInstance = new CoralDeploy();

	private final StructPublisher<Pose3d> poweredBarPublisher = NetworkTableInstance.getDefault()
			.getStructTopic("Mechanisms/Coral Deploy Powered Bar", Pose3d.struct)
			.publish();

	private final StructPublisher<Pose3d> unpoweredBarPublisher = NetworkTableInstance.getDefault()
			.getStructTopic("Mechanisms/Coral Deploy Unpowered Bar", Pose3d.struct)
			.publish();

	private final StructPublisher<Pose3d> mainIntakePublisher = NetworkTableInstance.getDefault()
			.getStructTopic("Mechanisms/Coral Deploy Main", Pose3d.struct)
			.publish();

	public CoralDeploy() {
		super(
				CoralDeployConstants.getMotorIO(),
				"Coral Deploy",
				CoralDeployConstants.kEpsilonThreshold,
				CoralDeployConstants.getServoHomingConfig());
		setCurrentPosition(CoralDeployConstants.kFullStowPosition);
		applySetpoint(STOW_CLEAR);
	}

	@Override
	public void outputTelemetry() {
		Angle poweredBarAngle = getPosition().unaryMinus();
		Angle unpoweredBarAngle = poweredBarAngle;
		Pose3d poweredBarPoseFacingLength = new Pose3d(
						CoralDeployConstants.kPoweredBarOffsetPose.getTranslation(), new Rotation3d())
				.plus(new Transform3d(
						new Translation3d(),
						new Rotation3d(BaseUnits.AngleUnit.zero(), poweredBarAngle, BaseUnits.AngleUnit.zero())));
		Pose3d unpoweredBarPoseFacingLength = new Pose3d(
						CoralDeployConstants.kUnpoweredBarOffsetPose.getTranslation(), new Rotation3d())
				.plus(new Transform3d(
						new Translation3d(),
						new Rotation3d(BaseUnits.AngleUnit.zero(), unpoweredBarAngle, BaseUnits.AngleUnit.zero())));

		Translation3d poweredBarEndTranslation = poweredBarPoseFacingLength
				.transformBy(new Transform3d(
						CoralDeployConstants.poweredBarDistance,
						BaseUnits.DistanceUnit.zero(),
						BaseUnits.DistanceUnit.zero(),
						Rotation3d.kZero))
				.getTranslation();
		Translation3d unpoweredBarEndTranslation = unpoweredBarPoseFacingLength
				.transformBy(new Transform3d(
						CoralDeployConstants.unpoweredBarDistance,
						BaseUnits.DistanceUnit.zero(),
						BaseUnits.DistanceUnit.zero(),
						Rotation3d.kZero))
				.getTranslation();
		Translation3d deltaTranslation = poweredBarEndTranslation.minus(unpoweredBarEndTranslation);
		Rotation2d mainIntakeAngle = new Rotation2d(deltaTranslation.getX(), deltaTranslation.getZ());

		poweredBarPublisher.set(poweredBarPoseFacingLength.plus(
				new Transform3d(new Translation3d(), CoralDeployConstants.kPoweredBarOffsetPose.getRotation())));
		unpoweredBarPublisher.set(unpoweredBarPoseFacingLength.plus(
				new Transform3d(new Translation3d(), CoralDeployConstants.kUnpoweredBarOffsetPose.getRotation())));
		mainIntakePublisher.set(
				new Pose3d(poweredBarEndTranslation, new Rotation3d(0.0, mainIntakeAngle.getRadians(), 0.0)));

		super.outputTelemetry();
	}
}
