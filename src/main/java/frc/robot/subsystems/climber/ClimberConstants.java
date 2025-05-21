package frc.robot.subsystems.climber;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.BaseUnits;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import frc.lib.io.MotorIOTalonFX;
import frc.lib.io.MotorIOTalonFX.MotorIOTalonFXConfig;
import frc.lib.io.MotorIOTalonFXSim;
import frc.lib.sim.LinearSim;
import frc.lib.sim.LinearSim.LinearSimConstants;
import frc.lib.util.Util;
import frc.robot.Ports;
import frc.robot.Robot;
import frc.robot.RobotConstants;

public class ClimberConstants {
	public static final double kGearing = RobotConstants.isComp ? (36.0 / 1.0) : (60.0 / 1.0);

	public static final Util.DistanceAngleConverter converter =
			new Util.DistanceAngleConverter(Units.Inches.of(0.675).div(2.0));

	public static final Distance kPrepPosition = Units.Inches.of(RobotConstants.isComp ? 18.78 : 17.78);
	public static final Distance kPullPosition = Units.Inches.of(10.948);
	public static final Distance kStowPosition = Units.Inches.of(10.948);
	public static final Distance kClearPosition = kStowPosition.plus(Units.Inches.of(0.5));

	public static final Angle kMaxExtension = (Units.Degrees.of(1600));

	public static final Distance kEpsilonThreshold = Units.Centimeter.of(5.0);

	public static final Pose3d kOffsetPose = new Pose3d(
			new Translation3d(-0.017112, -0.250703, 0.192849),
			new Rotation3d(Units.Degrees.of(-61.0), BaseUnits.AngleUnit.zero(), BaseUnits.AngleUnit.zero()));

	public static final Translation2d KPivotPointTranslation =
			new Translation2d(kOffsetPose.getY(), kOffsetPose.getZ());
	public static final Translation2d kPulleyPointTranslation = new Translation2d(-0.140696, 0.070612);

	public static final Distance kPivotPointToStringAttatchment = Units.Meters.of(0.124455);

	public static final TalonFXConfiguration getFXConfig() {
		TalonFXConfiguration config = new TalonFXConfiguration();
		config.Slot0.kP = 300.0;
		config.Slot0.kD = 0.0;
		config.Slot0.kS = 0.0;
		config.Slot0.kG = 0.0;
		config.Slot0.GravityType = GravityTypeValue.Elevator_Static;

		config.MotionMagic.MotionMagicAcceleration = 1000;
		config.MotionMagic.MotionMagicCruiseVelocity = 500;
		config.MotionMagic.MotionMagicJerk = 100;

		config.CurrentLimits.SupplyCurrentLimitEnable = Robot.isReal();
		config.CurrentLimits.SupplyCurrentLimit = 80.0;
		config.CurrentLimits.SupplyCurrentLowerLimit = -80.0;
		config.CurrentLimits.SupplyCurrentLowerTime = 0.1;

		config.Voltage.PeakForwardVoltage = 12.0;
		config.Voltage.PeakReverseVoltage = -12.0;

		config.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
		config.SoftwareLimitSwitch.ForwardSoftLimitThreshold =
				converter.toAngle(kStowPosition).plus(kMaxExtension).in(Units.Rotations);

		config.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
		config.SoftwareLimitSwitch.ReverseSoftLimitThreshold =
				converter.toAngle(kPullPosition).in(Units.Rotations);

		config.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
		config.Feedback.SensorToMechanismRatio = kGearing;
		config.MotorOutput.NeutralMode = NeutralModeValue.Brake;

		return config;
	}

	public static MotorIOTalonFXConfig getIOConfig() {
		MotorIOTalonFXConfig config = new MotorIOTalonFXConfig();
		config.mainConfig = getFXConfig();
		config.mainID = Ports.CLIMBER.id;
		config.mainBus = Ports.CLIMBER.bus;
		config.unit = converter.getDistanceUnitAsAngleUnit(Units.Inches);
		config.time = Units.Second;
		return config;
	}

	public static LinearSimConstants getSimConstants() {
		LinearSimConstants constants = new LinearSimConstants();
		constants.gearing = kGearing;
		constants.minHeight = kStowPosition;
		constants.maxHeight = kPrepPosition;
		constants.simGravity = false;
		constants.startingHeight = kStowPosition;
		constants.motor = DCMotor.getKrakenX60Foc(1);
		constants.carriageMass = Units.Pounds.of(12.0);
		constants.converter = converter;
		return constants;
	}

	public static MotorIOTalonFX getMotorIO() {
		if (Robot.isReal()) {
			return new MotorIOTalonFX(getIOConfig());
		} else {
			return new MotorIOTalonFXSim(getIOConfig(), new LinearSim(getSimConstants()));
		}
	}
}
