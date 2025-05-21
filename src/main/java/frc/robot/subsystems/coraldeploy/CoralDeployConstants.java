package frc.robot.subsystems.coraldeploy;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.StaticFeedforwardSignValue;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.BaseUnits;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import frc.lib.bases.ServoMotorSubsystem.ServoHomingConfig;
import frc.lib.io.MotorIOTalonFX;
import frc.lib.io.MotorIOTalonFX.MotorIOTalonFXConfig;
import frc.lib.io.MotorIOTalonFXSim;
import frc.lib.sim.PivotSim;
import frc.lib.sim.PivotSim.PivotSimConstants;
import frc.robot.Ports;
import frc.robot.Robot;
import frc.robot.RobotConstants;

public class CoralDeployConstants {
	public static final double kGearing = 40.0;

	public static final Angle kDeployPosition = Units.Degrees.of(RobotConstants.isOmega ? 10.0 : 3.0);
	public static final Angle kStowClearPosition = Units.Degrees.of(55.0);
	public static final Angle kFullStowPosition = Units.Degrees.of(RobotConstants.isOmega ? 85.918 : 83.0);
	public static final Angle kIndexerHold = Units.Degrees.of(RobotConstants.isOmega ? 20.0 : 10.0);

	public static final Angle kExhaustPosition = kDeployPosition;
	public static final Distance kArmLength = Units.Inches.of(14.0);

	public static final Angle kEpsilonThreshold = Units.Degrees.of(6.0);

	public static final Pose3d kPoweredBarOffsetPose = new Pose3d(
			Units.Meters.of(0.1406525),
			Units.Meters.of(0.0),
			Units.Meters.of(0.23622),
			new Rotation3d(BaseUnits.AngleUnit.zero(), Units.Degrees.of(-90.0), BaseUnits.AngleUnit.zero()));
	public static final Pose3d kUnpoweredBarOffsetPose = new Pose3d(
			Units.Meters.of(0.2791203968),
			Units.Meters.of(0.0),
			Units.Meters.of(0.1820658792),
			new Rotation3d(BaseUnits.AngleUnit.zero(), Units.Degrees.of(-90.0), BaseUnits.AngleUnit.zero()));
	public static final Pose3d kMainIntakeOffsetPose = new Pose3d(
			Units.Meters.of(0.0),
			Units.Meters.of(0.0),
			Units.Meters.of(0.0),
			new Rotation3d(BaseUnits.AngleUnit.zero(), Units.Degrees.of(0.0), BaseUnits.AngleUnit.zero()));

	public static final Distance unpoweredBarDistance = Units.Inches.of(12.6744953704);
	public static final Distance poweredBarDistance = Units.Inches.of(13.2062952857);

	public static TalonFXConfiguration getFXConfig() {
		TalonFXConfiguration config = new TalonFXConfiguration();
		config.Slot0.kP = 180.0;
		config.Slot0.kD = 0.0;
		config.Slot0.kS = 0.0;
		config.Slot0.kG = 0.0;

		config.Slot0.GravityType = GravityTypeValue.Arm_Cosine;
		config.Slot0.StaticFeedforwardSign = StaticFeedforwardSignValue.UseVelocitySign;

		config.MotionMagic.MotionMagicCruiseVelocity = 7.0;
		config.MotionMagic.MotionMagicAcceleration = 15.0;

		config.Voltage.PeakForwardVoltage = 12.0;
		config.Voltage.PeakReverseVoltage = -12.0;

		config.CurrentLimits.SupplyCurrentLimitEnable = true;
		config.CurrentLimits.SupplyCurrentLimit = 40.0;

		config.Feedback.SensorToMechanismRatio = kGearing;

		config.MotorOutput.NeutralMode = NeutralModeValue.Brake;

		config.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
		config.SoftwareLimitSwitch.ForwardSoftLimitThreshold = kFullStowPosition.in(Units.Rotations);

		config.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
		config.SoftwareLimitSwitch.ReverseSoftLimitThreshold = kDeployPosition.in(Units.Rotations);
		return config;
	}

	public static MotorIOTalonFXConfig getIOConfig() {
		MotorIOTalonFXConfig config = new MotorIOTalonFXConfig();
		config.mainConfig = getFXConfig();
		config.mainID = Ports.CORAL_DEPLOY.id;
		config.mainBus = Ports.CORAL_DEPLOY.bus;
		config.time = Units.Seconds;
		config.unit = Units.Degrees;
		return config;
	}

	public static MotorIOTalonFX getMotorIO() {
		if (Robot.isReal()) {
			return new MotorIOTalonFX(getIOConfig());
		} else {
			return new MotorIOTalonFXSim(getIOConfig(), new PivotSim(getSimConstants()));
		}
	}

	public static PivotSimConstants getSimConstants() {
		PivotSimConstants simConstants = new PivotSimConstants();
		simConstants.gearing = kGearing;
		simConstants.armLength = kArmLength;
		simConstants.momentOfInertia = Units.KilogramSquareMeters.of(0.6046665376);
		simConstants.motor = DCMotor.getKrakenX60Foc(1);
		simConstants.mechanismMaxHardStop = kFullStowPosition;
		simConstants.mechanismMinHardStop = kDeployPosition;
		simConstants.simGravity = false;
		simConstants.mechanismStartPos = kFullStowPosition;

		return simConstants;
	}

	public static ServoHomingConfig getServoHomingConfig() {
		ServoHomingConfig config = new ServoHomingConfig();
		config.kHomePosition = kDeployPosition;
		config.kHomingTimeout = Units.Seconds.of(0.2);
		config.kHomingVoltage = Units.Volts.of(-1.0);
		config.kSetHomedVelocity = Units.DegreesPerSecond.of(1.0);

		return config;
	}
}
