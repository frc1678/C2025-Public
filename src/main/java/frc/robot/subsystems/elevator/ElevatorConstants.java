package frc.robot.subsystems.elevator;

import com.ctre.phoenix6.configs.SoftwareLimitSwitchConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.BaseUnits;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Distance;
import frc.lib.bases.ServoMotorSubsystem.ServoHomingConfig;
import frc.lib.io.MotorIOTalonFX;
import frc.lib.io.MotorIOTalonFX.MotorIOTalonFXConfig;
import frc.lib.io.MotorIOTalonFXSim;
import frc.lib.sim.LinearSim;
import frc.lib.sim.LinearSim.LinearSimConstants;
import frc.lib.util.Util;
import frc.robot.Ports;
import frc.robot.Robot;

public class ElevatorConstants {

	public static final double kGearing = (3.0 / 1.0);

	public static final Util.DistanceAngleConverter converter = new Util.DistanceAngleConverter(
			Units.Inches.of(2.0).plus(Units.Inches.of(0.125)).div(2.0));

	public static final Pose3d stage1Offset = new Pose3d(-0.152, -0.165, -0.910, new Rotation3d());
	public static final Pose3d stage2Offset = new Pose3d(
			-0.0,
			0.0,
			0.0,
			new Rotation3d(BaseUnits.AngleUnit.zero(), BaseUnits.AngleUnit.zero(), Units.Degrees.of(90.0)));

	public static final Distance kMaxHeight = converter.toDistance(Units.Degrees.of(3316));

	public static final Distance kAlgaeLiftDistance = Units.Inches.of(2.0);

	public static final Distance kNetHeight = Units.Inches.of(62.17);
	public static final Distance kL1ScoringHeight = Units.Inches.of(0.0);
	public static final Distance kL2ScoringHeight = Units.Inches.of(6.3);
	public static final Distance kL3ScoringHeight = kL2ScoringHeight.plus(Units.Inches.of(16.0));
	public static final Distance kL4ScoringHeight = Units.Inches.of(60.25);
	public static final Distance kL2AlgaePosition = Units.Inches.of(21.91);
	public static final Distance kL3AlgaePosition = kL2AlgaePosition.plus(Units.Inches.of(16.0));
	public static final Distance kL2AlgaeLiftPosition = kL2AlgaePosition.plus(kAlgaeLiftDistance);
	public static final Distance kL3AlgaeLiftPosition = kL3AlgaePosition.plus(kAlgaeLiftDistance);

	public static final Distance kClearLowPosition = Units.Inches.of(12.0);
	public static final Distance kClearHighPosition = Units.Inches.of(18.608);
	public static final Distance kAlgaeFeedPosition = Units.Inches.of(2.00);
	public static final Distance kAlageHoldPosition = Units.Inches.of(19.41);
	public static final Distance kCoralHoldPosition = Units.Inches.of(13.21);
	public static final Distance kProcessorScoringHeight = Units.Inches.of(2.0);
	public static final Distance kL4PivotClearHeight = kL4ScoringHeight.minus(Units.Inches.of(32.0));
	public static final Distance kNetPrepHeight = Units.Inches.of(30.0);
	public static final Distance kEEUnjameHeight = Units.Inches.of(6.0);

	public static final Distance kStationIntake = Units.Inches.of(7.1);

	public static final Distance kLolipop = Units.Inches.of(6.0);

	public static final Distance kClimbPosition = Units.Inches.of(12.0);

	public static final Distance kStowPosition = Units.Inches.of(0.0);

	public static final Distance kEpsilonThreshold = Units.Inches.of(1.0);

	public static final Distance kElevatorHighThreshold = kCoralHoldPosition.plus(kEpsilonThreshold);

	public static final TalonFXConfiguration getFXConfig() {
		TalonFXConfiguration FXConfig = new TalonFXConfiguration();
		FXConfig.Slot0.kP = 16.3;
		FXConfig.Slot0.kD = 0.5;
		FXConfig.Slot0.kS = 0.45;
		FXConfig.Slot0.kG = 0.35;

		FXConfig.MotionMagic.MotionMagicCruiseVelocity = 20.0;

		FXConfig.CurrentLimits.SupplyCurrentLimitEnable = Robot.isReal();
		FXConfig.CurrentLimits.SupplyCurrentLimit = 80.0;
		FXConfig.CurrentLimits.SupplyCurrentLowerLimit = 80.0;
		FXConfig.CurrentLimits.SupplyCurrentLowerTime = 0.1;

		FXConfig.CurrentLimits.StatorCurrentLimitEnable = true;
		FXConfig.CurrentLimits.StatorCurrentLimit = 120.0;

		FXConfig.Voltage.PeakForwardVoltage = 12.0;
		FXConfig.Voltage.PeakReverseVoltage = -12.0;

		FXConfig.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
		FXConfig.SoftwareLimitSwitch.ForwardSoftLimitThreshold =
				converter.toAngle(kNetHeight).in(Units.Rotations);

		FXConfig.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
		FXConfig.SoftwareLimitSwitch.ReverseSoftLimitThreshold =
				converter.toAngle(kStowPosition).minus(Units.Degrees.of(10.0)).in(Units.Rotations);

		FXConfig.Feedback.SensorToMechanismRatio = kGearing;

		FXConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;

		return FXConfig;
	}

	public static MotorIOTalonFXConfig getIOConfig() {
		MotorIOTalonFXConfig IOConfig = new MotorIOTalonFXConfig();
		IOConfig.mainConfig = getFXConfig();
		IOConfig.mainID = Ports.ELEVATOR_MAIN.id;
		IOConfig.mainBus = Ports.ELEVATOR_MAIN.bus;
		IOConfig.unit = converter.getDistanceUnitAsAngleUnit(Units.Inches);
		IOConfig.time = Units.Second;
		IOConfig.followerConfig = getFXConfig()
				.withSoftwareLimitSwitch(new SoftwareLimitSwitchConfigs()
						.withForwardSoftLimitEnable(false)
						.withReverseSoftLimitEnable(false));
		IOConfig.followerOpposeMain = new boolean[] {false};
		IOConfig.followerBuses = new String[] {Ports.ELEVATOR_FOLLOWER.bus};
		IOConfig.followerIDs = new int[] {Ports.ELEVATOR_FOLLOWER.id};
		return IOConfig;
	}

	public static LinearSimConstants getSimConstants() {
		LinearSimConstants simConstants = new LinearSimConstants();
		simConstants.motor = DCMotor.getKrakenX60Foc(1);
		simConstants.gearing = kGearing;
		simConstants.carriageMass = Units.Pounds.of(27);
		simConstants.startingHeight = kStowPosition;
		simConstants.minHeight = kStowPosition;
		simConstants.maxHeight = kNetHeight;
		simConstants.simGravity = false;
		simConstants.converter = converter;
		return simConstants;
	}

	public static MotorIOTalonFX getMotorIO() {
		if (Robot.isReal()) {
			return new MotorIOTalonFX(getIOConfig());
		} else {
			return new MotorIOTalonFXSim(getIOConfig(), new LinearSim(getSimConstants()));
		}
	}

	public static ServoHomingConfig getServoConfig() {
		ServoHomingConfig servoConfig = new ServoHomingConfig();
		servoConfig.kHomePosition = converter.toAngle(kStowPosition);
		servoConfig.kHomingTimeout = Units.Seconds.of(0.5);
		servoConfig.kHomingVoltage = Units.Volts.of(-0.5);
		servoConfig.kSetHomedVelocity = converter.toAngle(Units.Inches.of(0.1)).per(Units.Second);

		return servoConfig;
	}
}
