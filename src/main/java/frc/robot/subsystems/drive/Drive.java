package frc.robot.subsystems.drive;

import choreo.trajectory.SwerveSample;
import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.swerve.SwerveDrivetrain.SwerveDriveState;
import com.ctre.phoenix6.swerve.SwerveRequest;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.units.BaseUnits;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import java.util.function.UnaryOperator;

public class Drive extends SubsystemBase {
	public static final Drive mInstance = new Drive();

	private SwerveDriveState lastReadState;
	private SwerveRequest driveRequest = DriveConstants.teleopRequest;
	private final GeneratedDrivetrain drivetrain = GeneratedConstants.createDrivetrain();
	private final GeneratedTelemetry telemetry = new GeneratedTelemetry(DriveConstants.kMaxSpeed.baseUnitMagnitude());
	private Time lastPoseResetTime = BaseUnits.TimeUnit.of(0.0);

	private StructPublisher<Pose3d> mechanismPublisher = NetworkTableInstance.getDefault()
			.getStructTopic("Mechanisms/Drivetrain", Pose3d.struct)
			.publish();

	private final Field2d elasticPose = new Field2d();

	private Drive() {
		lastReadState = drivetrain.getState();
		drivetrain.setDefaultCommand(drivetrain.applyRequest(() -> {
			return driveRequest;
		}));

		if (!Robot.isReal()) {
			drivetrain.resetPose((new Pose2d(new Translation2d(), Rotation2d.fromDegrees(90.0))));
		}

		drivetrain.getOdometryThread().setThreadPriority(31);
	}

	public GeneratedDrivetrain getGeneratedDrive() {
		return drivetrain;
	}

	@Override
	public void periodic() {
		lastReadState = drivetrain.getState();
		outputTelemetry();
	}

	public void outputTelemetry() {
		mechanismPublisher.set(new Pose3d(getPose()));
		telemetry.telemeterize(lastReadState);
		SmartDashboard.putData("Drive", this);
		elasticPose.setRobotPose(getPose());
		SmartDashboard.putData("Elastic Field 2D", elasticPose);
	}

	@Override
	public void initSendable(SendableBuilder builder) {
		builder.addDoubleProperty(
				"Pitch Velocity Degrees Per Second",
				() -> drivetrain
						.getPigeon2()
						.getAngularVelocityYDevice()
						.getValue()
						.in(Units.DegreesPerSecond),
				null);
		builder.addDoubleProperty(
				"Pitch Degrees",
				() -> drivetrain.getPigeon2().getPitch().getValue().in(Units.Degrees),
				null);

		builder.addDoubleProperty(
				"Roll Velocity Degrees Per Second",
				() -> drivetrain
						.getPigeon2()
						.getAngularVelocityXDevice()
						.getValue()
						.in(Units.DegreesPerSecond),
				null);
		builder.addDoubleProperty(
				"Roll Degrees",
				() -> drivetrain.getPigeon2().getRoll().getValue().in(Units.Degrees),
				null);

		addModuleToBuilder(builder, 0);
		addModuleToBuilder(builder, 1);
		addModuleToBuilder(builder, 2);
		addModuleToBuilder(builder, 3);
	}

	private void addModuleToBuilder(SendableBuilder builder, int module) {
		builder.addDoubleProperty(
				"ModuleStates/" + module + "/Drive/Volts",
				() -> drivetrain
						.getModules()[module]
						.getDriveMotor()
						.getMotorVoltage()
						.getValue()
						.in(Units.Volts),
				null);

		builder.addDoubleProperty(
				"ModuleStates/" + module + "/Rotation/Volts",
				() -> drivetrain
						.getModules()[module]
						.getSteerMotor()
						.getMotorVoltage()
						.getValue()
						.in(Units.Volts),
				null);

		builder.addDoubleProperty(
				"ModuleStates/" + module + "/Drive/Stator Current",
				() -> drivetrain
						.getModules()[module]
						.getDriveMotor()
						.getStatorCurrent()
						.getValue()
						.in(Units.Amps),
				null);

		builder.addDoubleProperty(
				"ModuleStates/" + module + "/Drive/Temperature Celsius",
				() -> drivetrain
						.getModules()[module]
						.getDriveMotor()
						.getDeviceTemp()
						.getValue()
						.in(Units.Celsius),
				null);

		builder.addDoubleProperty(
				"ModuleStates/" + module + "/Rotation/Stator Current",
				() -> drivetrain
						.getModules()[module]
						.getSteerMotor()
						.getStatorCurrent()
						.getValue()
						.in(Units.Amps),
				null);

		builder.addDoubleProperty(
				"ModuleStates/" + module + "/Drive/Supply Current",
				() -> drivetrain
						.getModules()[module]
						.getDriveMotor()
						.getSupplyCurrent()
						.getValue()
						.in(Units.Amps),
				null);

		builder.addDoubleProperty(
				"ModuleStates/" + module + "/Rotation/Supply Current",
				() -> drivetrain
						.getModules()[module]
						.getSteerMotor()
						.getSupplyCurrent()
						.getValue()
						.in(Units.Amps),
				null);

		builder.addDoubleProperty(
				"ModuleStates/" + module + "/Rotation/Temperature Celsius",
				() -> drivetrain
						.getModules()[module]
						.getSteerMotor()
						.getDeviceTemp()
						.getValue()
						.in(Units.Celsius),
				null);
	}

	public SwerveDriveState getState() {
		return drivetrain.getState();
	}

	public Pose2d getPose() {
		return lastReadState.Pose;
	}

	public void setSwerveRequest(SwerveRequest request) {
		driveRequest = request;
	}

	public Command followSwerveRequestCommand(
			SwerveRequest.FieldCentric request, UnaryOperator<SwerveRequest.FieldCentric> updater) {
		return run(() -> setSwerveRequest(updater.apply(request)))
				.handleInterrupt(() -> setSwerveRequest(new SwerveRequest.FieldCentric()));
	}

	public void followChoreoTrajectory(SwerveSample sample) {
		setSwerveRequest(
				DriveConstants.getPIDToPoseRequestUpdater(sample.getPose()).apply(DriveConstants.PIDToPoseRequest));
	}

	public void addVisionUpdate(Pose2d pose, Time timestamp) {
		getGeneratedDrive().addVisionMeasurement(pose, timestamp.in(Units.Seconds));
	}

	public void addVisionUpdate(Pose2d pose, Time timestamp, Matrix<N3, N1> stdDevs) {
		getGeneratedDrive().addVisionMeasurement(pose, timestamp.in(Units.Seconds), stdDevs);
	}

	public void resetPose(Pose2d pose) {
		getGeneratedDrive().resetPose(pose);
		lastPoseResetTime =
				Units.Seconds.of(Utils.getCurrentTimeSeconds()).plus(DriveConstants.updatePreventTimePostPoseReset);
	}

	public Command resetPoseCmd(Pose2d pose) {
		return Commands.runOnce(() -> resetPose(pose));
	}

	public boolean getPitchStable() {
		return drivetrain.getPigeon2().getAngularVelocityYDevice().getValue().abs(Units.DegreesPerSecond)
						< DriveConstants.maxVelocityStableThreshold.in(Units.DegreesPerSecond)
				&& drivetrain.getPigeon2().getPitch().getValue().abs(BaseUnits.AngleUnit)
						< DriveConstants.maxPitchStableThreshold.in(BaseUnits.AngleUnit);
	}

	public boolean getRollStable() {
		return drivetrain.getPigeon2().getAngularVelocityXDevice().getValue().abs(Units.DegreesPerSecond)
						< DriveConstants.maxVelocityStableThreshold.in(Units.DegreesPerSecond)
				&& drivetrain.getPigeon2().getRoll().getValue().abs(BaseUnits.AngleUnit)
						< DriveConstants.maxPitchStableThreshold.in(BaseUnits.AngleUnit);
	}

	public boolean getStable() {
		ChassisSpeeds speeds = getState().Speeds;
		return getPitchStable()
				&& getRollStable()
				&& Units.MetersPerSecond.of(Math.hypot(speeds.vxMetersPerSecond, speeds.vyMetersPerSecond))
						.lte(DriveConstants.kScoringTranslationMaxSpeed)
				&& Units.RadiansPerSecond.of(speeds.omegaRadiansPerSecond).lte(DriveConstants.kScoringRotationMaxSpeed);
	}
}
