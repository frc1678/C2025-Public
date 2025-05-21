package frc.robot.subsystems.drive;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularAcceleration;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearAcceleration;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.lib.logging.LogUtil;
import frc.lib.util.FieldLayout.Level;
import frc.lib.util.SynchronousPIDF;
import frc.lib.util.Util;
import frc.robot.RobotConstants;
import frc.robot.controlboard.ControlBoardConstants;
import java.util.function.UnaryOperator;

public class DriveConstants {
	public static final LinearVelocity kMaxSpeed = GeneratedConstants.kSpeedAt12Volts;
	public static final LinearVelocity kMaxSpeedFAST = kMaxSpeed.times(2.0);

	public static final LinearAcceleration kMaxAcceleration = Units.MetersPerSecondPerSecond.of(12.0);
	public static final AngularVelocity kMaxAngularRate = Units.RadiansPerSecond.of(2.75 * Math.PI);
	public static final AngularVelocity kMaxAngularRateFAST = kMaxAngularRate.times(2.0);
	public static final AngularAcceleration kMaxAngularAcceleration =
			kMaxAngularRate.div(0.1).per(Units.Second);

	public static final LinearVelocity kMaxSpeedTippy = kMaxSpeed.div(2.0);
	public static final LinearVelocity kMaxSpeedVeryTippy = kMaxSpeed.div(2.9);

	public static final LinearVelocity kChoreoMaxSpeed = kMaxSpeed.times(0.85);
	public static final LinearAcceleration kChoreoMaxAcceleration = kMaxAcceleration.times(0.7);

	public static final LinearVelocity kDetectionMaxSpeed = kMaxSpeed.times(0.5);

	public static final LinearVelocity kScoringTranslationMaxSpeed =
			Units.Centimeters.of(15.0).per(Units.Seconds);
	public static final AngularVelocity kScoringRotationMaxSpeed =
			Units.Degrees.of(7.0).per(Units.Seconds);

	public static void exitController(ProfiledPIDController controller) {
		controller.reset(controller.getGoal());
	}

	private static SynchronousPIDF getAutoAlignHeadingController() {
		SynchronousPIDF controller = new SynchronousPIDF(5.0, 0.0, 0.0);
		controller.setInputRange(-0.5, 0.5);
		controller.setMaxAbsoluteOutput(kMaxAngularRate.in(Units.RotationsPerSecond));
		controller.setContinuous();
		return controller;
	}

	private static SynchronousPIDF getAutoAlignTranslationController() {
		SynchronousPIDF controller = new SynchronousPIDF(3.15, 0.0, 0.0);
		controller.setMaxAbsoluteOutput(kMaxSpeed.in(Units.MetersPerSecond));
		return controller;
	}

	private static SynchronousPIDF getAutoAlignTippyHeadingController() {
		SynchronousPIDF controller = new SynchronousPIDF(5.0, 0.0, 0.0);
		controller.setInputRange(-0.5, 0.5);
		controller.setMaxAbsoluteOutput(kMaxAngularRate.times(0.5).in(Units.RotationsPerSecond));
		controller.setContinuous();
		return controller;
	}

	private static SynchronousPIDF getAutoAlignTippyTranslationController() {
		SynchronousPIDF controller = new SynchronousPIDF(2.6, 0.0, 0.0);
		controller.setMaxAbsoluteOutput(kMaxSpeedTippy.in(Units.MetersPerSecond));
		return controller;
	}

	private static SynchronousPIDF getAutoAlignVeryTippyHeadingController() {
		SynchronousPIDF controller = new SynchronousPIDF(5.2, 0.0, 0.0);
		controller.setInputRange(-0.5, 0.5);
		controller.setMaxAbsoluteOutput(kMaxAngularRate.times(0.25).in(Units.RotationsPerSecond));
		controller.setContinuous();
		return controller;
	}

	private static SynchronousPIDF getAutoAlignVeryTippyTranslationController() {
		SynchronousPIDF controller = new SynchronousPIDF(2.8, 0.0, 0.0);
		controller.setMaxAbsoluteOutput(kMaxSpeedVeryTippy.in(Units.MetersPerSecond));
		return controller;
	}

	private static SynchronousPIDF getChoreoHeadingController() {
		SynchronousPIDF controller = new SynchronousPIDF(5.2, 0.0, 0.0);
		controller.setInputRange(0, 360);
		return controller;
	}

	private static SynchronousPIDF getChoreoTranslationController() {
		SynchronousPIDF controller = new SynchronousPIDF(3.0, 0.0, 0.0);
		controller.setMaxAbsoluteOutput(kMaxSpeed.in(Units.MetersPerSecond));
		return controller;
	}

	public static final SynchronousPIDF mAutoAlignHeadingController = getAutoAlignHeadingController();
	public static final SynchronousPIDF mAutoAlignTranslationController = getAutoAlignTranslationController();

	public static final SynchronousPIDF mAutoHeadingController = getAutoAlignHeadingController();
	public static final SynchronousPIDF mAutoTranslationController = getAutoAlignTranslationController();

	public static final SynchronousPIDF mChoreoHeadingController = getChoreoHeadingController();
	public static final SynchronousPIDF mChoreoTranslationController = getChoreoTranslationController();

	public static final SynchronousPIDF mStayOnLineTranslationController = getAutoAlignTippyTranslationController();
	public static final SynchronousPIDF mStayOnLineHeadingController = getAutoAlignTippyHeadingController();

	public static final SwerveRequest.FieldCentric teleopRequest =
			new SwerveRequest.FieldCentric().withDriveRequestType(DriveRequestType.OpenLoopVoltage);

	public static final UnaryOperator<SwerveRequest.FieldCentric> teleopRequestUpdater =
			(SwerveRequest.FieldCentric request) -> {
				double xDesiredRaw = -ControlBoardConstants.mDriverController.getLeftY();
				double yDesiredRaw = -ControlBoardConstants.mDriverController.getLeftX();
				double rotDesiredRaw = -ControlBoardConstants.mDriverController.getRightX();
				double xFancy = getDeadbandedStick(xDesiredRaw);
				double yFancy = getDeadbandedStick(yDesiredRaw);
				double rotFancy = getDeadbandedStick(rotDesiredRaw);

				SmartDashboard.putNumber("Sticks/hypot/raw", Math.hypot(xDesiredRaw, yDesiredRaw));

				return request.withVelocityX((ControlBoardConstants.mDriverController
												.leftStick()
												.getAsBoolean()
										? DriveConstants.kMaxSpeedFAST
										: DriveConstants.kMaxSpeed)
								.times(xFancy))
						.withVelocityY((ControlBoardConstants.mDriverController
												.leftStick()
												.getAsBoolean()
										? DriveConstants.kMaxSpeedFAST
										: DriveConstants.kMaxSpeed)
								.times(yFancy))
						.withRotationalRate((ControlBoardConstants.mDriverController
												.rightStick()
												.getAsBoolean()
										? DriveConstants.kMaxAngularRateFAST
										: DriveConstants.kMaxAngularRate)
								.times(rotFancy));
			};

	public static double getDeadbandedStick(double rawValue) {
		if (Math.abs(rawValue) < ControlBoardConstants.stickDeadband) {
			return 0.0;
		} else {
			double unsignedValue = (Math.abs(rawValue) - ControlBoardConstants.stickDeadband)
					/ (1.0 - ControlBoardConstants.stickDeadband);
			return (rawValue > 0 ? unsignedValue : -unsignedValue);
		}
	}

	public static final SwerveRequest.FieldCentric PIDToPoseRequest = new SwerveRequest.FieldCentric()
			.withDeadband(DriveConstants.kMaxSpeed
					.times(ControlBoardConstants.stickDeadband)
					.div(10.0))
			.withRotationalDeadband(DriveConstants.kMaxAngularRate
					.times(ControlBoardConstants.stickDeadband)
					.div(10.0))
			.withDriveRequestType(DriveRequestType.Velocity);

	public static final SwerveRequest.FieldCentric AutoPIDToPoseRequest = new SwerveRequest.FieldCentric()
			.withDeadband(DriveConstants.kMaxSpeed
					.times(ControlBoardConstants.stickDeadband)
					.div(40.0))
			.withRotationalDeadband(DriveConstants.kMaxAngularRate
					.times(ControlBoardConstants.stickDeadband)
					.div(40.0))
			.withDriveRequestType(DriveRequestType.Velocity);

	public static final UnaryOperator<SwerveRequest.FieldCentric> getPIDToPoseRequestUpdater(Pose2d targetPose) {
		return getPIDToPoseRequestUpdater(targetPose, mAutoAlignTranslationController, mAutoAlignHeadingController);
	}

	public static final UnaryOperator<SwerveRequest.FieldCentric> getPIDToPoseRequestUpdater(
			Pose2d targetPose, SynchronousPIDF translationController, SynchronousPIDF headingController) {
		return (SwerveRequest.FieldCentric request) -> {
			Pose2d currentPose = Drive.mInstance.getPose();
			Translation2d deltaTranslation = currentPose.getTranslation().minus(targetPose.getTranslation());
			LinearVelocity velocity =
					Units.MetersPerSecond.of(translationController.calculate(deltaTranslation.getNorm()));
			Rotation2d velocityDirection = deltaTranslation.getAngle();
			if (RobotConstants.isRedAlliance) velocityDirection = velocityDirection.plus(Rotation2d.k180deg);

			headingController.setSetpoint(Units.Radians.of(
							MathUtil.angleModulus(targetPose.getRotation().getRadians()))
					.in(Units.Rotations));

			LogUtil.recordPose2d("PID To Pose Updater/Target Pose", targetPose);

			request.withVelocityX(velocity.times(velocityDirection.getCos()))
					.withVelocityY(velocity.times(velocityDirection.getSin()))
					.withRotationalRate(Units.RotationsPerSecond.of(
							headingController.calculate(Units.Radians.of(MathUtil.angleModulus(
											currentPose.getRotation().getRadians()))
									.in(Units.Rotations))));

			return request;
		};
	}

	public static final UnaryOperator<SwerveRequest.FieldCentric> getPIDToTranslationForwardRequestUpdater(
			Pose2d targetPose) {
		return (SwerveRequest.FieldCentric request) -> {
			Pose2d currentPose = Drive.mInstance.getPose();
			Translation2d deltaTranslation = currentPose.getTranslation().minus(targetPose.getTranslation());
			LinearVelocity velocity =
					Units.MetersPerSecond.of(mAutoAlignTranslationController.calculate(deltaTranslation.getNorm()));
			Rotation2d velocityDirection = deltaTranslation.getAngle();
			if (RobotConstants.isRedAlliance) velocityDirection = velocityDirection.plus(Rotation2d.k180deg);

			return request.withVelocityX(velocity.times(velocityDirection.getCos()))
					.withVelocityY(velocity.times(velocityDirection.getSin()))
					.withRotationalRate(Units.DegreesPerSecond.of(mAutoAlignHeadingController.calculate(currentPose
							.getRotation()
							.minus(deltaTranslation.getAngle().plus(Rotation2d.k180deg))
							.getDegrees())));
		};
	}

	public static final SwerveRequest.FieldCentric ChoreoRequest = new SwerveRequest.FieldCentric()
			.withDeadband(DriveConstants.kMaxSpeed
					.times(ControlBoardConstants.stickDeadband)
					.div(10.0))
			.withRotationalDeadband(DriveConstants.kMaxAngularRate
					.times(ControlBoardConstants.stickDeadband)
					.div(10.0))
			.withDriveRequestType(DriveRequestType.Velocity);

	public static final UnaryOperator<SwerveRequest.FieldCentric> getChoreoRequestUpdater(Pose2d targetPose) {
		return (SwerveRequest.FieldCentric request) -> {
			Pose2d currentPose = Drive.mInstance.getPose();
			Translation2d deltaTranslation = currentPose.getTranslation().minus(targetPose.getTranslation());
			LinearVelocity velocity =
					Units.MetersPerSecond.of(mChoreoTranslationController.calculate(deltaTranslation.getNorm()));
			Rotation2d velocityDirection = deltaTranslation.getAngle();
			if (RobotConstants.isRedAlliance) velocityDirection = velocityDirection.plus(Rotation2d.k180deg);

			return request.withVelocityX(velocity.times(velocityDirection.getCos()))
					.withVelocityY(velocity.times(velocityDirection.getSin()))
					.withRotationalRate(Units.DegreesPerSecond.of(mChoreoHeadingController.calculate(currentPose
							.getRotation()
							.minus(targetPose.getRotation())
							.getDegrees())));
		};
	}

	public static final SwerveRequest.RobotCentric RobotCentricRequest =
			new SwerveRequest.RobotCentric().withDriveRequestType(DriveRequestType.Velocity);

	public static TrajectoryConfig getTrajectoryConfig() {
		return new TrajectoryConfig(DriveConstants.kMaxSpeed, DriveConstants.kMaxAcceleration);
	}

	public static final Time updatePreventTimePostPoseReset = Units.Seconds.of(0.1);

	public static final AngularVelocity maxVelocityStableThreshold = Units.DegreesPerSecond.of(10.0);
	public static final Angle maxPitchStableThreshold = Units.Degrees.of(5.0);

	public static final LinearAcceleration kMaxAccelTippy = kMaxAcceleration.div(3.0);
	public static final LinearAcceleration kMaxAccelVeryTippy = kMaxAcceleration.div(5.0);

	public static final SynchronousPIDF mAutoAlignTippyHeadingController = getAutoAlignTippyHeadingController();
	public static final SynchronousPIDF mAutoAlignTippyTranslationController = getAutoAlignTippyTranslationController();

	public static final SynchronousPIDF mAutoAlignVeryTippyHeadingController = getAutoAlignVeryTippyHeadingController();
	public static final SynchronousPIDF mAutoAlignVeryTippyTranslationController =
			getAutoAlignVeryTippyTranslationController();

	public static final Time elevatorRaiseTimeNet = Units.Seconds.of(0.75);
	public static final Distance distanceToStartSlowingNet = Units.Meters.of(Util.calculateDistanceToStartDeccel(
			kMaxSpeed.in(Units.MetersPerSecond),
			kMaxAcceleration.in(Units.MetersPerSecondPerSecond),
			kMaxAccelVeryTippy.in(Units.MetersPerSecondPerSecond),
			elevatorRaiseTimeNet.in(Units.Seconds)));
	public static final Distance distanceToRaiseElevatorNet = Units.Meters.of(Util.calculatDistanceToRaiseElevator(
			kMaxAccelVeryTippy.in(Units.MetersPerSecondPerSecond), elevatorRaiseTimeNet.in(Units.Seconds)));

	public static final Time elevatorRaiseTimeL4 = Units.Seconds.of(0.65);
	public static final Distance distanceToStartSlowingL4 = Units.Meters.of(Util.calculateDistanceToStartDeccel(
			kMaxSpeed.in(Units.MetersPerSecond),
			kMaxAcceleration.in(Units.MetersPerSecondPerSecond),
			kMaxAccelTippy.in(Units.MetersPerSecondPerSecond),
			elevatorRaiseTimeL4.in(Units.Seconds)));
	public static final Distance distanceToRaiseElevatorL4 = Units.Meters.of(Util.calculatDistanceToRaiseElevator(
			kMaxAccelTippy.in(Units.MetersPerSecondPerSecond), elevatorRaiseTimeL4.in(Units.Seconds)));

	public static final Distance distanceToStartSlowingL3 = Units.Meters.of(999.0);
	public static final Distance distanceToRaiseElevatorL3 = Units.Meters.of(999.0);

	public static final Distance distanceToStartSlowingL2 = Units.Meters.of(999.0);
	public static final Distance distanceToRaiseElevatorL2 = Units.Meters.of(999.0);

	public static final Time elevatorRaiseTimeL1 = Units.Seconds.of(0.4);
	public static final Distance distanceToStartSlowingL1 = Units.Meters.of(Util.calculateDistanceToStartDeccel(
			kMaxSpeed.in(Units.MetersPerSecond),
			kMaxAcceleration.in(Units.MetersPerSecondPerSecond),
			kMaxAcceleration.in(Units.MetersPerSecondPerSecond),
			elevatorRaiseTimeL1.in(Units.Seconds)));
	public static final Distance distanceToRaiseElevatorL1 = Units.Meters.of(Util.calculatDistanceToRaiseElevator(
			kMaxAcceleration.in(Units.MetersPerSecondPerSecond), elevatorRaiseTimeL1.in(Units.Seconds)));

	public static final Distance distanceToStartSlowingL2Algae = Units.Meters.of(999.0);
	public static final Distance distanceToRaiseElevatorL2Algae = Units.Meters.of(999.0);

	public static final Distance distanceToStartSlowingL3Algae = Units.Meters.of(999.0);
	public static final Distance distanceToRaiseElevatorL3Algae = Units.Meters.of(999.0);

	public static final Time elevatorRaiseTimeProcessor = Units.Seconds.of(0.5);
	public static final Distance distanceToStartSlowingProcessorAlgae = Units.Meters.of(999.0);
	public static final Distance distanceToRaiseElevatorProcessorAlgae = Units.Meters.of(999.0);

	public static SynchronousPIDF getTippyTranslationControllerForLevel(Level level) {
		return switch (level) {
			case L1,
					L2,
					L2_ALGAE,
					L3,
					L3_ALGAE,
					PROCESSOR_ALGAE -> mAutoAlignTranslationController; // not actually tippy so just return normal
			case L4 -> mAutoAlignTippyTranslationController; // elevator is high so use tippy controller
			case NET -> mAutoAlignVeryTippyTranslationController;
			default -> null;
		};
	}

	public static SynchronousPIDF getTippyHeadingControllerForLevel(Level level) {
		return switch (level) {
			case L1,
					L2,
					L2_ALGAE,
					L3,
					L3_ALGAE,
					PROCESSOR_ALGAE -> mAutoAlignHeadingController; // not actually tippy so just return normal
			case L4 -> mAutoAlignTippyHeadingController; // elevator is high so use tippy controller
			case NET -> mAutoAlignVeryTippyHeadingController;
			default -> null;
		};
	}

	public static Distance getDistanceToStartSlowingForLevel(Level level) {
		return switch (level) {
			case L1 -> distanceToStartSlowingL1;
			case L2 -> distanceToStartSlowingL2;
			case L3 -> distanceToStartSlowingL3;
			case L4 -> distanceToStartSlowingL4;
			case L2_ALGAE -> distanceToStartSlowingL2Algae;
			case L3_ALGAE -> distanceToStartSlowingL3Algae;
			case PROCESSOR_ALGAE -> distanceToStartSlowingProcessorAlgae;
			case NET -> distanceToStartSlowingNet;
			default -> null;
		};
	}

	public static Distance getDistanceToRaiseElevatorForLevel(Level level) {
		return switch (level) {
			case L1 -> distanceToRaiseElevatorL1;
			case L2 -> distanceToRaiseElevatorL2;
			case L3 -> distanceToRaiseElevatorL3;
			case L4 -> distanceToRaiseElevatorL4;
			case L2_ALGAE -> distanceToRaiseElevatorL2Algae;
			case L3_ALGAE -> distanceToRaiseElevatorL3Algae;
			case PROCESSOR_ALGAE -> distanceToRaiseElevatorProcessorAlgae;
			case NET -> distanceToRaiseElevatorNet;
			default -> null;
		};
	}

	public static LinearVelocity getVelocityToRaiseForLevel(Level level) {
		return switch (level) {
			case L1, L2, L2_ALGAE, PROCESSOR_ALGAE -> kMaxSpeed.times(1.1); // not actually tippy so just return normal
			case L4, L3, L3_ALGAE -> kMaxSpeedTippy.times(1.1); // elevator is high so use tippy controller
			case NET -> kMaxSpeedVeryTippy.times(1.1);
			default -> null;
		};
	}
}
