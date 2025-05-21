package frc.robot.subsystems.elevator;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Voltage;
import frc.lib.bases.ServoMotorSubsystem;
import frc.lib.io.MotorIO.Setpoint;
import frc.lib.io.MotorIOTalonFX;

public class Elevator extends ServoMotorSubsystem<MotorIOTalonFX> {
	private final StructPublisher<Pose3d> stage1Publisher = NetworkTableInstance.getDefault()
			.getStructTopic("Mechanisms/Elevator Stage 1", Pose3d.struct)
			.publish();

	private final StructPublisher<Pose3d> stage2Publisher = NetworkTableInstance.getDefault()
			.getStructTopic("Mechanisms/Elevator Stage 2", Pose3d.struct)
			.publish();

	public static final Setpoint JOG_UP = Setpoint.withVoltageSetpoint(Voltage.ofBaseUnits(0.5, Units.Volts));
	public static final Setpoint JOG_DOWN = Setpoint.withVoltageSetpoint(Voltage.ofBaseUnits(-0.5, Units.Volts));
	public static final Setpoint HOLD_UP = Setpoint.withVoltageSetpoint(Voltage.ofBaseUnits(0, Units.Volts));

	public static final Setpoint L4_SCORE =
			Setpoint.withMotionMagicSetpoint(ElevatorConstants.converter.toAngle(ElevatorConstants.kL4ScoringHeight));
	public static final Setpoint L3_SCORE =
			Setpoint.withMotionMagicSetpoint(ElevatorConstants.converter.toAngle(ElevatorConstants.kL3ScoringHeight));
	public static final Setpoint L2_SCORE =
			Setpoint.withMotionMagicSetpoint(ElevatorConstants.converter.toAngle(ElevatorConstants.kL2ScoringHeight));
	public static final Setpoint L1_SCORE =
			Setpoint.withMotionMagicSetpoint(ElevatorConstants.converter.toAngle(ElevatorConstants.kL1ScoringHeight));
	public static final Setpoint NET_HEIGHT =
			Setpoint.withMotionMagicSetpoint(ElevatorConstants.converter.toAngle(ElevatorConstants.kNetHeight));
	public static final Setpoint L2_ALGAE =
			Setpoint.withMotionMagicSetpoint(ElevatorConstants.converter.toAngle(ElevatorConstants.kL2AlgaePosition));
	public static final Setpoint L3_ALGAE =
			Setpoint.withMotionMagicSetpoint(ElevatorConstants.converter.toAngle(ElevatorConstants.kL3AlgaePosition));
	public static final Setpoint CLEAR_LOW_HEIGHT =
			Setpoint.withMotionMagicSetpoint(ElevatorConstants.converter.toAngle(ElevatorConstants.kClearLowPosition));
	public static final Setpoint CLEAR_HIGH_HEIGHT =
			Setpoint.withMotionMagicSetpoint(ElevatorConstants.converter.toAngle(ElevatorConstants.kClearHighPosition));
	public static final Setpoint ALGAE_FEED_HEIGHT =
			Setpoint.withMotionMagicSetpoint(ElevatorConstants.converter.toAngle(ElevatorConstants.kAlgaeFeedPosition));
	public static final Setpoint ALGAE_HOLD =
			Setpoint.withMotionMagicSetpoint(ElevatorConstants.converter.toAngle(ElevatorConstants.kAlageHoldPosition));
	public static final Setpoint CORAL_HOLD =
			Setpoint.withMotionMagicSetpoint(ElevatorConstants.converter.toAngle(ElevatorConstants.kCoralHoldPosition));
	public static final Setpoint STOW =
			Setpoint.withMotionMagicSetpoint(ElevatorConstants.converter.toAngle(ElevatorConstants.kStowPosition));
	public static final Setpoint CLIMB =
			Setpoint.withMotionMagicSetpoint(ElevatorConstants.converter.toAngle(ElevatorConstants.kClimbPosition));
	public static final Setpoint LOLIPOP =
			Setpoint.withMotionMagicSetpoint(ElevatorConstants.converter.toAngle(ElevatorConstants.kLolipop));
	public static final Setpoint NET_PREP =
			Setpoint.withMotionMagicSetpoint(ElevatorConstants.converter.toAngle(ElevatorConstants.kNetPrepHeight));
	public static final Setpoint PROCESSOR = Setpoint.withMotionMagicSetpoint(
			ElevatorConstants.converter.toAngle(ElevatorConstants.kProcessorScoringHeight));
	public static final Setpoint L4_CLEAR = Setpoint.withMotionMagicSetpoint(
			ElevatorConstants.converter.toAngle(ElevatorConstants.kL4PivotClearHeight));
	public static final Setpoint L2_LIFT = Setpoint.withMotionMagicSetpoint(
			ElevatorConstants.converter.toAngle(ElevatorConstants.kL2AlgaeLiftPosition));
	public static final Setpoint L3_LIFT = Setpoint.withMotionMagicSetpoint(
			ElevatorConstants.converter.toAngle(ElevatorConstants.kL3AlgaeLiftPosition));
	public static final Setpoint EE_UNJAM =
			Setpoint.withMotionMagicSetpoint(ElevatorConstants.converter.toAngle(ElevatorConstants.kEEUnjameHeight));
	public static final Setpoint CORAL_STATION =
			Setpoint.withMotionMagicSetpoint(ElevatorConstants.converter.toAngle(ElevatorConstants.kStationIntake));

	public static final Elevator mInstance = new Elevator();

	private Elevator() {
		super(
				ElevatorConstants.getMotorIO(),
				"Elevator",
				ElevatorConstants.converter.toAngle(ElevatorConstants.kEpsilonThreshold),
				ElevatorConstants.getServoConfig());
		setCurrentPosition(ElevatorConstants.converter.toAngle(ElevatorConstants.kStowPosition));
		applySetpoint(STOW);
	}

	@Override
	public void outputTelemetry() {
		super.outputTelemetry();
		stage2Publisher.set(ElevatorConstants.stage2Offset.plus(new Transform3d(
				new Translation3d(
						0.0,
						0.0,
						ElevatorConstants.converter.toDistance(getPosition()).in(Units.Meters)),
				new Rotation3d(0.0, 0.0, 0.0))));
		stage1Publisher.set(ElevatorConstants.stage1Offset.plus(new Transform3d(
				new Translation3d(
						0.0,
						0.0,
						ElevatorConstants.converter
								.toDistance(getPosition())
								.div(2.0)
								.in(Units.Meters)),
				new Rotation3d(0.0, 0.0, 0.0))));
	}
}
