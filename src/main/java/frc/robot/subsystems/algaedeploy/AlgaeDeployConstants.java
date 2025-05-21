package frc.robot.subsystems.algaedeploy;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
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

public class AlgaeDeployConstants {
	public static final double kGearing = (62.0 / 8.0) * (68.0 / 18.0) * (15.0 / 9.0);

	public static final Angle kStowPosition = Units.Degrees.of(90.0);
	public static final Angle kClearPosition = Units.Degrees.of(65.0);
	public static final Angle kAlgaeClearPosition = Units.Degree.of(40.0);
	public static final Angle kDeployPosition = Units.Degrees.of(23.0);
	public static final Angle kProcessorPosition = Units.Degrees.of(70.0);
	public static final Angle kL1Score = Units.Degrees.of(55.0);
	public static final Angle kFarClearPosition = Units.Degrees.of(45.0);
	public static final Angle kUnjamEndEffectorAngle = Units.Degrees.of(30.0);

	public static final Distance kArmLength = Units.Inches.of(16.5);

	public static final Angle kEpsilonThreshold = Units.Degrees.of(8.0);

	public static final Pose3d kOffsetPose = new Pose3d(
			Units.Meters.of(-0.302971),
			Units.Meters.of(0.0),
			Units.Meters.of(0.177800),
			new Rotation3d(BaseUnits.AngleUnit.zero(), Units.Degrees.of(90.0), BaseUnits.AngleUnit.zero()));

	public static TalonFXConfiguration getFXConfig() {
		TalonFXConfiguration config = new TalonFXConfiguration();
		config.Slot0.kP = 115.0;
		config.Slot0.kS = 0.0;
		config.Slot0.kG = 0.2;

		config.Slot0.GravityType = GravityTypeValue.Arm_Cosine;

		config.MotionMagic.MotionMagicCruiseVelocity = 100.0;
		config.MotionMagic.MotionMagicAcceleration = 80.0;

		config.Voltage.PeakForwardVoltage = 12.0;
		config.Voltage.PeakReverseVoltage = -12.0;

		config.CurrentLimits.SupplyCurrentLimitEnable = true;
		config.CurrentLimits.SupplyCurrentLimit = 40.0;
		config.CurrentLimits.SupplyCurrentLowerLimit = 40.0;
		config.CurrentLimits.SupplyCurrentLowerTime = 0.1;

		config.Feedback.SensorToMechanismRatio = kGearing;

		config.MotorOutput.NeutralMode = NeutralModeValue.Brake;

		config.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
		config.SoftwareLimitSwitch.ForwardSoftLimitThreshold = kStowPosition.in(Units.Rotations);

		config.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
		config.SoftwareLimitSwitch.ReverseSoftLimitThreshold = -1000;

		config.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

		return config;
	}

	public static MotorIOTalonFXConfig getIOConfig() {
		MotorIOTalonFXConfig config = new MotorIOTalonFXConfig();
		config.mainConfig = getFXConfig();
		config.mainID = Ports.ALGAE_DEPLOY.id;
		config.mainBus = Ports.ALGAE_DEPLOY.bus;
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
		simConstants.momentOfInertia = Units.KilogramSquareMeters.of(0.1324882135);
		simConstants.motor = new DCMotor(
				12, 4.05, 275, 1.4, edu.wpi.first.math.util.Units.rotationsPerMinuteToRadiansPerSecond(7530), 1);
		simConstants.mechanismMaxHardStop = kStowPosition;
		simConstants.mechanismMinHardStop = Units.Degrees.of(0.0);
		simConstants.simGravity = false;
		simConstants.mechanismStartPos = kStowPosition;

		return simConstants;
	}

	public static ServoHomingConfig getServoHomingConfig() {
		ServoHomingConfig config = new ServoHomingConfig();
		config.kHomePosition = kStowPosition;
		config.kHomingTimeout = Units.Seconds.of(0.2);
		config.kHomingVoltage = Units.Volts.of(1.0);
		config.kSetHomedVelocity = Units.DegreesPerSecond.of(5.0);

		return config;
	}
}
