package frc.robot.subsystems.pivot;

import com.ctre.phoenix6.hardware.CANcoder;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.units.BaseUnits;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.util.sendable.SendableBuilder;
import frc.lib.bases.ServoMotorSubsystem;
import frc.lib.io.MotorIO.Setpoint;
import frc.lib.io.MotorIOTalonFX;
import frc.robot.Ports;
import frc.robot.Robot;
import frc.robot.RobotConstants;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.elevator.ElevatorConstants;

public class Pivot extends ServoMotorSubsystem<MotorIOTalonFX> {
	public static final Setpoint CORAL_INTAKE = Setpoint.withMotionMagicSetpoint(PivotConstants.kCoralIntake);
	public static final Setpoint ALGAE_INTAKE = Setpoint.withMotionMagicSetpoint(PivotConstants.kAlgaeIntake);
	public static final Setpoint PROCESSOR = Setpoint.withMotionMagicSetpoint(PivotConstants.kProcessorScore);
	public static final Setpoint REEF_INTAKE = Setpoint.withMotionMagicSetpoint(PivotConstants.kReefIntake);
	public static final Setpoint REEF_PREP = Setpoint.withMotionMagicSetpoint(PivotConstants.kReefPrep);

	public static final Setpoint L1_SCORE = Setpoint.withMotionMagicSetpoint(PivotConstants.kL1Score);
	public static final Setpoint L2_SCORE = Setpoint.withMotionMagicSetpoint(PivotConstants.kL2Score);
	public static final Setpoint L3_SCORE = Setpoint.withMotionMagicSetpoint(PivotConstants.kL3Score);
	public static final Setpoint L4_SCORE = Setpoint.withMotionMagicSetpoint(PivotConstants.kL4Score);
	public static final Setpoint NET_SCORE = Setpoint.withMotionMagicSetpoint(PivotConstants.kNetScore);

	public static final Setpoint ALGAE_HOLD = Setpoint.withMotionMagicSetpoint(PivotConstants.kAlgaeHold);
	public static final Setpoint CORAL_HOLD = Setpoint.withMotionMagicSetpoint(PivotConstants.kCoralHold);

	public static final Setpoint CLIMB = Setpoint.withMotionMagicSetpoint(PivotConstants.kClimb);

	public static final Setpoint JOG_POSITIVE = Setpoint.withVoltageSetpoint(Units.Volts.of(1.0));
	public static final Setpoint JOG_NEGATIVE = Setpoint.withVoltageSetpoint(Units.Volts.of(-1.0));
	public static final Setpoint HOLD = Setpoint.withNeutralSetpoint();

	public static final Setpoint CORAL_STATION = Setpoint.withMotionMagicSetpoint(PivotConstants.kStationIntake);

	public static final Pivot mInstance = new Pivot();

	private CANcoder gearedCancoder;
	public CANcoder directCancoder;

	private Debouncer resetDebouncer = new Debouncer(0.1, DebounceType.kRising);
	private boolean lastShouldReset = false;

	public Pivot() {
		super(
				PivotConstants.getMotorIO(),
				"Pivot",
				RobotConstants.isOmega ? Units.Degrees.of(2.0) : Units.Degrees.of(2.0));
		if (Robot.isReal()) {
			directCancoder = new CANcoder(Ports.ENCODER_41T.id, Ports.ENCODER_41T.bus);
			directCancoder.getConfigurator().apply(PivotConstants.getDirect41TCancoderConfig());

			gearedCancoder = new CANcoder(Ports.ENCODER_39T.id, Ports.ENCODER_39T.bus);
			gearedCancoder.getConfigurator().apply(PivotConstants.getGeared39TCancoderConfig());

			directCancoder.setPosition(directCancoder.getAbsolutePosition().getValue());
			setCurrentPosition(directCancoder.getAbsolutePosition().getValue());
		} else {
			setCurrentPosition(PivotConstants.kCoralIntake);
		}
	}

	public static final StructPublisher<Pose3d> publisher = NetworkTableInstance.getDefault()
			.getStructTopic("Mechanisms/Pivot", Pose3d.struct)
			.publish();

	@Override
	public void outputTelemetry() {
		super.outputTelemetry();

		try {
			publisher.set(PivotConstants.kOffsetPose.plus(new Transform3d(
					new Translation3d(
							BaseUnits.DistanceUnit.zero(),
							BaseUnits.DistanceUnit.zero(),
							ElevatorConstants.converter.toDistance(Elevator.mInstance.getPosition())),
					new Rotation3d(BaseUnits.AngleUnit.zero(), getPosition(), BaseUnits.AngleUnit.zero()))));
		} catch (Exception e) {
			// TODO: handle exception
		}
	}

	@Override
	public void initSendable(SendableBuilder builder) {
		super.initSendable(builder);

		if (Robot.isReal()) {
			builder.addDoubleProperty(
					"Direct encoder (41T)",
					() -> directCancoder.getPosition().getValue().in(Units.Degrees),
					null);
			builder.addDoubleProperty(
					"Geared encoder (39T)",
					() -> gearedCancoder.getPosition().getValue().in(Units.Degrees),
					null);
			builder.addDoubleProperty(
					"Abs Position Degrees", () -> getAbsolutePosition().in(Units.Degrees), null);

			builder.addBooleanProperty("Last Should Reset", () -> lastShouldReset, null);
		}
	}

	public Angle getAbsolutePosition() {
		Angle positionRemainder = directCancoder.getAbsolutePosition().getValue();
		Angle gearedEncoderPos = gearedCancoder.getAbsolutePosition().getValue();
		Angle gearedEncoder0RotsPosition = positionRemainder.times(PivotConstants.kGearedCancoderGearing);
		Angle diff = gearedEncoderPos.minus(gearedEncoder0RotsPosition);
		Angle diffRemainder = Units.Radians.of(MathUtil.angleModulus(diff.in(Units.Radians)));
		long fullRotations =
				Math.round(diffRemainder.in(Units.Rotations) / (PivotConstants.kGearedCancoderGearing - 1.0));
		Angle absolutePosition = positionRemainder.plus(Units.Rotations.of(fullRotations));
		return absolutePosition;
	}
}
