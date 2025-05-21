package frc.lib.bases;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.bases.LimelightSubsystem.LimelightConfig;
import frc.lib.io.DetectionIOLimelight;
import frc.lib.logging.LogUtil;
import frc.lib.logging.LoggedTracer;
import frc.lib.util.LimelightHelpers;
import frc.robot.Robot;
import frc.robot.subsystems.drive.Drive;

public class DetectionSubsystem<IO extends DetectionIOLimelight> extends SubsystemBase {
	protected final IO io;
	private LimelightConfig config = new LimelightConfig();

	public DetectionSubsystem(LimelightConfig config, IO io) {
		this.io = io;
		this.config = config;
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

			io.configLimelight(config);
		}
	}

	@Override
	public void periodic() {
		io.update();
		outputTelemetry();
	}

	public boolean getDisabled() {
		return io.getDisabled();
	}

	/**
	 * @return The closest coral pose.
	 */
	public Pose2d getCoralPose() {
		return io.getCoralPose(Drive.mInstance.getPose().getTranslation());
	}

	/**
	 * @param base The translation to evaluate the closest coral relative to
	 *
	 * @return The closest coral pose.
	 */
	public Pose2d getCoralPose(Translation2d base) {
		return io.getCoralPose(base);
	}

	/**
	 * @param base The translation to evaluate the closest coral relative to
	 *
	 * @return A pose with a rotation component equal to the angle to face the coral and the translation component of the closest coral.
	 */
	public Pose2d getCoralTranslationAndPoint(Translation2d base) {
		Translation2d t = getCoralPose(base).getTranslation();
		Rotation2d r = t.minus(Drive.mInstance.getPose().getTranslation()).getAngle();
		LogUtil.recordPose2d("Detection PID/Coral Translation And Point", new Pose2d(t, r));
		return new Pose2d(t, r);
	}

	/**
	 * @return A pose with a rotation component equal to the angle to face the coral and the translation component of the closest coral based on the drivetrain.
	 */
	public Pose2d getCoralTranslationAndPoint() {
		return getCoralTranslationAndPoint(Drive.mInstance.getPose().getTranslation());
	}

	public int coralCount() {
		return io.coralCount();
	}

	public void setPipeline(int index) {
		io.setPipeline(index);
	}

	public boolean hasCoral() {
		return getCoralPose() != null;
	}

	public void outputTelemetry() {
		LoggedTracer.record(config.name);
	}

	@Override
	public void initSendable(SendableBuilder builder) {
		builder.addBooleanProperty(config.name + "/Has Coral", () -> hasCoral(), null);
		builder.addDoubleProperty(
				config.name + "/Latest Pipeline Index",
				() -> LimelightHelpers.getCurrentPipelineIndex(config.name),
				null);
	}
}
