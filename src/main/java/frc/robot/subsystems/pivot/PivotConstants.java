package frc.robot.subsystems.pivot;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.BaseUnits;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import frc.lib.io.MotorIOTalonFX;
import frc.lib.io.MotorIOTalonFX.MotorIOTalonFXConfig;
import frc.lib.io.MotorIOTalonFXSim;
import frc.lib.sim.PivotSim;
import frc.lib.sim.PivotSim.PivotSimConstants;
import frc.robot.Ports;
import frc.robot.Robot;
import frc.robot.RobotConstants;

public class PivotConstants {
	public static final double kGearing = (48.0 / 10.0) * (64.0 / 18.0) * (48.0 / 18.0);

	public static final Angle kL1Score = Units.Degrees.of(102.0);
	public static final Angle kL2Score = Units.Degrees.of(100.75);
	public static final Angle kL3Score = Units.Degrees.of(100.75);
	public static final Angle kL4Score = Units.Degrees.of(140.0);
	public static final Angle kNetScore = Units.Degrees.of(-218.0);

	public static final Angle kStationIntake = Units.Degrees.of(80.0);
	public static final Angle kAutoStart = Units.Degrees.of(90.0);

	public static final Angle kCoralIntake = Robot.isReal() ? Units.Degrees.of(-90.0) : Units.Degrees.of(0.0);
	public static final Angle kAlgaeIntake = Units.Degrees.of(-90.0);
	public static final Angle kReefIntake = Units.Degrees.of(-100.0);
	public static final Angle kReefPrep = Units.Degrees.of(-60.0);

	public static final Angle kProcessorScore = Units.Degrees.of(-90.0);

	public static final Angle kEndEffectorIdleAfterScoringAngle = Units.Degrees.of(-45.0);

	public static final Angle kAlgaeHold = Units.Degrees.of(-120.0);
	public static final Angle kCoralHold = Units.Degrees.of(60.0);

	public static final Angle kClimb = Units.Degrees.of(130.0);

	public static final Angle kAlgaeImpactAngle = Units.Degrees.of(140.0);
	public static final Angle kCoralImpactAngle = Units.Degrees.of(40.0);

	public static final Distance kOutElevatorOffset = Units.Inches.of(9.5);

	public static final double kGearedCancoderGearing = 41.0 / 39.0;

	public static final int kRotationsToCheck = 10;

	public static final Pose3d kOffsetPose = new Pose3d(
			new Translation3d(-0.1778, 0.0, 0.47),
			new Rotation3d(BaseUnits.AngleUnit.zero(), BaseUnits.AngleUnit.zero(), BaseUnits.AngleUnit.zero()));

	public static TalonFXConfiguration getFXConfig() {
		TalonFXConfiguration config = new TalonFXConfiguration();
		config.Slot0.kP = RobotConstants.isOmega ? 100.0 : 75.0;
		config.Slot0.kD = RobotConstants.isOmega ? 2.0 : 2.5;
		config.Slot0.kS = RobotConstants.isOmega ? 0.35 : 0.05;

		config.Slot0.kG = 0.2;

		config.Slot0.GravityType = GravityTypeValue.Arm_Cosine;

		config.MotionMagic.MotionMagicCruiseVelocity = 2.0;
		config.MotionMagic.MotionMagicAcceleration = 1.0;

		config.Voltage.PeakForwardVoltage = 12.0;
		config.Voltage.PeakReverseVoltage = -12.0;

		config.CurrentLimits.StatorCurrentLimitEnable = Robot.isReal();
		config.CurrentLimits.StatorCurrentLimit = 60.0;

		config.CurrentLimits.SupplyCurrentLimitEnable = Robot.isReal();
		config.CurrentLimits.SupplyCurrentLimit = 30.0;
		config.CurrentLimits.SupplyCurrentLowerLimit = 30.0;
		config.CurrentLimits.SupplyCurrentLowerTime = 0.1;

		config.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
		config.SoftwareLimitSwitch.ForwardSoftLimitThreshold =
				Units.Rotations.of(999.0).in(Units.Rotations);

		config.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
		config.SoftwareLimitSwitch.ReverseSoftLimitThreshold =
				Units.Rotations.of(-999.0).in(Units.Rotations);
		config.MotorOutput.NeutralMode = NeutralModeValue.Brake;

		config.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

		config.Feedback.SensorToMechanismRatio = kGearing;
		return config;
	}

	public static MotorIOTalonFXConfig getIOConfig() {
		MotorIOTalonFXConfig config = new MotorIOTalonFXConfig();
		config.mainBus = Ports.PIVOT.bus;
		config.mainID = Ports.PIVOT.id;
		config.time = Units.Seconds;
		config.unit = Units.Degrees;
		config.mainConfig = getFXConfig();
		return config;
	}

	public static PivotSimConstants getSimConstants() {
		PivotSimConstants simConstants = new PivotSimConstants();
		simConstants.gearing = kGearing;
		simConstants.momentOfInertia = Units.KilogramSquareMeters.of(0.111571209);
		simConstants.motor = new DCMotor(
				12, 4.05, 275, 1.4, edu.wpi.first.math.util.Units.rotationsPerMinuteToRadiansPerSecond(7530), 1);
		simConstants.armLength = kOutElevatorOffset;
		simConstants.mechanismMinHardStop = Units.Rotations.of(-999.0);
		simConstants.mechanismMaxHardStop = Units.Rotations.of(999.0);
		simConstants.simGravity = false;
		simConstants.mechanismStartPos = kCoralIntake;
		return simConstants;
	}

	public static MotorIOTalonFX getMotorIO() {
		if (Robot.isReal()) {
			return new MotorIOTalonFX(getIOConfig());
		} else {
			return new MotorIOTalonFXSim(getIOConfig(), new PivotSim(getSimConstants()));
		}
	}

	public static CANcoderConfiguration getGeared39TCancoderConfig() {
		CANcoderConfiguration config = new CANcoderConfiguration();
		config.MagnetSensor.AbsoluteSensorDiscontinuityPoint = 0.5;
		config.MagnetSensor.MagnetOffset = (RobotConstants.isComp ? -0.262451 : 0.2275390625) - 0.25;
		config.MagnetSensor.SensorDirection = SensorDirectionValue.CounterClockwise_Positive;
		return config;
	}

	public static CANcoderConfiguration getDirect41TCancoderConfig() {
		CANcoderConfiguration config = new CANcoderConfiguration();
		config.MagnetSensor.AbsoluteSensorDiscontinuityPoint = 0.5;
		config.MagnetSensor.MagnetOffset = (RobotConstants.isComp ? -0.381592 : 0.368896484375) - 0.25;
		config.MagnetSensor.SensorDirection = SensorDirectionValue.Clockwise_Positive;
		return config;
	}
}
