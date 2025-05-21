package frc.robot.subsystems.algaedeploy;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.units.Units;
import frc.lib.bases.ServoMotorSubsystem;
import frc.lib.io.MotorIO.Setpoint;
import frc.lib.io.MotorIOTalonFX;

public class AlgaeDeploy extends ServoMotorSubsystem<MotorIOTalonFX> {
	private StructPublisher<Pose3d> publisher = NetworkTableInstance.getDefault()
			.getStructTopic("Mechanisms/Algae Deploy", Pose3d.struct)
			.publish();

	public static final Setpoint DEPLOY = Setpoint.withMotionMagicSetpoint(AlgaeDeployConstants.kDeployPosition);
	public static final Setpoint CLEAR = Setpoint.withMotionMagicSetpoint(AlgaeDeployConstants.kClearPosition);
	public static final Setpoint STOW = Setpoint.withMotionMagicSetpoint(AlgaeDeployConstants.kStowPosition);
	public static final Setpoint PROCESSOR = Setpoint.withMotionMagicSetpoint(AlgaeDeployConstants.kProcessorPosition);
	public static final Setpoint L1_SCORE = Setpoint.withMotionMagicSetpoint(AlgaeDeployConstants.kL1Score);
	public static final Setpoint FAR_CLEAR = Setpoint.withMotionMagicSetpoint(AlgaeDeployConstants.kFarClearPosition);
	public static final Setpoint ALGAE_CLEAR =
			Setpoint.withMotionMagicSetpoint(AlgaeDeployConstants.kAlgaeClearPosition);
	public static final Setpoint EE_UNJAM =
			Setpoint.withMotionMagicSetpoint(AlgaeDeployConstants.kUnjamEndEffectorAngle);

	public static final AlgaeDeploy mInstance = new AlgaeDeploy();

	public AlgaeDeploy() {
		super(
				AlgaeDeployConstants.getMotorIO(),
				"Algae Deploy",
				Units.Degrees.of(3.0),
				AlgaeDeployConstants.getServoHomingConfig());
		setCurrentPosition(AlgaeDeployConstants.kStowPosition);
		applySetpoint(STOW);
	}

	@Override
	public void outputTelemetry() {
		super.outputTelemetry();
		publisher.set(AlgaeDeployConstants.kOffsetPose.plus(new Transform3d(
				new Translation3d(), new Rotation3d(0.0, getPosition().in(Units.Radians), 0.0))));
	}
}
