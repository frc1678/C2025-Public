package frc.lib.bases;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.io.VisionIOLimelight;
import frc.lib.logging.LoggedTracer;
import frc.lib.util.LimelightHelpers;
import frc.robot.Robot;

public class LimelightSubsystem<IO extends VisionIOLimelight> extends SubsystemBase {
	protected final IO io;

	public LimelightSubsystem(LimelightConfig config, IO io) {
		super(config.name);
		this.io = io;

		if (Robot.isReal()) {
			LimelightHelpers.setCameraPose_RobotSpace(
					config.name,
					config.robotToCameraOffset.getX(),
					config.robotToCameraOffset.getY(),
					config.robotToCameraOffset.getZ(),
					Units.radiansToDegrees(
							config.robotToCameraOffset.getRotation().getX()),
					Units.radiansToDegrees(
							config.robotToCameraOffset.getRotation().getY()),
					Units.radiansToDegrees(
							config.robotToCameraOffset.getRotation().getZ()));
		}
		this.io.updateConfig(config);
	}

	@Override
	public void periodic() {
		io.update();
		outputTelemetry();
	}

	public void disable(boolean disable) {
		io.disable(disable);
	}

	public boolean getDisabled() {
		return io.getDisabled();
	}

	public void outputTelemetry() {
		LoggedTracer.record("Vision");
	}

	public static class LimelightConfig {
		public String name = "no-name-assigned";
		public Pose3d robotToCameraOffset = new Pose3d();
		public Vector<N3> aprilTagVisionStdDevs = VecBuilder.fill(0.3, 0.3, 99999.0);
	}
}
