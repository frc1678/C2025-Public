package frc.lib.util;

import edu.wpi.first.math.Pair;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.interpolation.InterpolatingTreeMap;
import edu.wpi.first.math.interpolation.Interpolator;
import edu.wpi.first.math.interpolation.InverseInterpolator;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.Trajectory.State;
import edu.wpi.first.units.AngleUnit;
import edu.wpi.first.units.BaseUnits;
import edu.wpi.first.units.DistanceUnit;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Unit;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.RobotConstants;
import frc.robot.subsystems.drive.Drive;
import java.util.ArrayList;
import java.util.Collection;
import java.util.List;
import java.util.Set;
import java.util.concurrent.atomic.AtomicInteger;
import java.util.concurrent.atomic.AtomicReference;
import java.util.function.IntSupplier;
import java.util.function.Supplier;

/**
 * Contains basic functions that are used often.
 */
public class Util {
	public static final double kEpsilon = 1e-12;

	/**
	 * Prevent this class from being instantiated.
	 */
	private Util() {}

	/**
	 * Limits the given input to the given magnitude.
	 */
	public static double limit(double v, double maxMagnitude) {
		return limit(v, -maxMagnitude, maxMagnitude);
	}

	public static double limit(double v, double min, double max) {
		return Math.min(max, Math.max(min, v));
	}

	public static int limit(int v, int min, int max) {
		return Math.min(max, Math.max(min, v));
	}

	public static boolean inRange(double v, double maxMagnitude) {
		return inRange(v, -maxMagnitude, maxMagnitude);
	}

	/**
	 * Checks if the given input is within the range (min, max), both exclusive.
	 */
	public static boolean inRange(double v, double min, double max) {
		return v > min && v < max;
	}

	public static double interpolate(double a, double b, double x) {
		x = limit(x, 0.0, 1.0);
		return a + (b - a) * x;
	}

	public static String joinStrings(final String delim, final List<?> strings) {
		StringBuilder sb = new StringBuilder();
		for (int i = 0; i < strings.size(); ++i) {
			sb.append(strings.get(i).toString());
			if (i < strings.size() - 1) {
				sb.append(delim);
			}
		}
		return sb.toString();
	}

	public static boolean epsilonEquals(double a, double b, double epsilon) {
		return (a - epsilon <= b) && (a + epsilon >= b);
	}

	public static boolean epsilonEquals(double a, double b) {
		return epsilonEquals(a, b, kEpsilon);
	}

	public static boolean epsilonEquals(int a, int b, int epsilon) {
		return (a - epsilon <= b) && (a + epsilon >= b);
	}

	public static boolean epsilonEquals(Translation2d a, Translation2d b) {
		return epsilonEquals(a.getX(), b.getX()) || epsilonEquals(a.getY(), b.getY());
	}

	public static boolean epsilonEquals(Translation2d a, Translation2d b, double epsilon) {
		return epsilonEquals(a.getX(), b.getX(), epsilon) && epsilonEquals(a.getY(), b.getY(), epsilon);
	}

	public static boolean epsilonEquals(Translation2d a, Translation2d b, Distance epsilon) {
		return epsilonEquals(a.getX(), b.getX(), epsilon.in(Units.Meters))
				&& epsilonEquals(a.getY(), b.getY(), epsilon.in(Units.Meters));
	}

	public static boolean epsilonEquals(ChassisSpeeds a, ChassisSpeeds b) {
		return epsilonEquals(a.vxMetersPerSecond, b.vxMetersPerSecond)
				&& epsilonEquals(a.vyMetersPerSecond, b.vyMetersPerSecond)
				&& epsilonEquals(a.omegaRadiansPerSecond, b.omegaRadiansPerSecond);
	}

	public static boolean epsilonEquals(ChassisSpeeds a, ChassisSpeeds b, double linearVelocityEpsilon) {
		return epsilonEquals(a.vxMetersPerSecond, b.vxMetersPerSecond, linearVelocityEpsilon)
				&& epsilonEquals(a.vyMetersPerSecond, b.vyMetersPerSecond, linearVelocityEpsilon);
	}

	public static boolean allCloseTo(final List<Double> list, double value, double epsilon) {
		boolean result = true;
		for (Double value_in : list) {
			result &= epsilonEquals(value_in, value, epsilon);
		}
		return result;
	}

	public static double handleDeadband(double value, double deadband) {
		deadband = Math.abs(deadband);
		if (deadband == 1) {
			return 0;
		}
		double scaledValue = (value + (value < 0 ? deadband : -deadband)) / (1 - deadband);
		return (Math.abs(value) > Math.abs(deadband)) ? scaledValue : 0;
	}

	public static Translation3d flipRedBlue(Translation3d original) {
		return new Translation3d(
				FieldLayout.kFieldLength.minus(original.getMeasureX()), original.getMeasureY(), original.getMeasureZ());
	}

	public static <T> Supplier<T> memoizeByIteration(IntSupplier iteration, Supplier<T> delegate) {
		AtomicReference<T> value = new AtomicReference<>();
		AtomicInteger last_iteration = new AtomicInteger(-1);
		return () -> {
			int last = last_iteration.get();
			int now = iteration.getAsInt();
			if (last != now) {
				value.updateAndGet((cur) -> null);
			}
			T val = value.get();
			if (val == null) {
				val = value.updateAndGet(cur -> cur == null ? delegate.get() : cur);
				last_iteration.set(now);
			}
			return val;
		};
	}

	/**
	 * Class used store translate distances in the form of angles. Used for elevators to interface with the IO layer which only supports angles.
	 */
	public static class DistanceAngleConverter {
		private final Distance radius;

		public DistanceAngleConverter(Distance radius) {
			this.radius = radius;
		}

		/**
		 * Converts a distance measurement to an equal angle measurement based on radius initialized with.
		 *
		 * @param distance Distance to convert to angle.
		 * @return Angle distance is equivalent to.
		 */
		public Angle toAngle(Distance distance) {
			return Units.Radians.of(distance.in(BaseUnits.DistanceUnit) / radius.baseUnitMagnitude());
		}

		/**
		 * Converts an angle measurement to an equal distance measurement based on radius initialized with.
		 *
		 * @param distance angle to convert to distance.
		 * @return Distance agle is equivalent to.
		 */
		public Distance toDistance(Angle angle) {
			return BaseUnits.DistanceUnit.of(angle.in(Units.Radians) * radius.baseUnitMagnitude());
		}

		/**
		 * Gets an angle unit equivalent to a distance unit with the conversion of the radius initialized with.
		 *
		 * @param unit The distance unit to convert.
		 * @return The distance represented as an AngleUnit
		 */
		public AngleUnit getDistanceUnitAsAngleUnit(DistanceUnit unit) {
			return Units.derive(BaseUnits.AngleUnit)
					.aggregate(toAngle(unit.one()).baseUnitMagnitude())
					.named(unit.name())
					.symbol(unit.symbol())
					.make();
		}

		/**
		 * Gets a distance unit equivalent to a angle unit with the conversion of the radius initialized with.
		 *
		 * @param unit The angle unit to convert.
		 * @return The distance represented as a DistanceUnit
		 */
		public DistanceUnit getAngleUnitAsDistanceUnit(AngleUnit unit) {
			return Units.derive(BaseUnits.DistanceUnit)
					.splitInto(toDistance(unit.one()).baseUnitMagnitude())
					.named(unit.name())
					.symbol(unit.symbol())
					.make();
		}

		public Distance getDrumRadius() {
			return radius;
		}
	}

	public static class MeasureInterpolable<T extends Measure<U>, U extends Unit> implements Interpolator<T> {
		@Override
		@SuppressWarnings("unchecked")
		public T interpolate(T startValue, T endValue, double t) {
			return (T) startValue.plus(endValue.minus(startValue).times(t));
		}
	}

	public static class MeasureInverseInterpolable<T extends Measure<U>, U extends Unit>
			implements InverseInterpolator<T> {
		@Override
		public double inverseInterpolate(T startValue, T endValue, T q) {
			return q.minus(startValue).baseUnitMagnitude()
					/ endValue.minus(startValue).baseUnitMagnitude();
		}
	}

	public static class InterpolatingMeasureMap<
					J extends Measure<U>, U extends Unit, K extends Measure<Q>, Q extends Unit>
			extends InterpolatingTreeMap<J, K> {
		public InterpolatingMeasureMap() {
			super(new MeasureInverseInterpolable<J, U>(), new MeasureInterpolable<K, Q>());
		}

		public InterpolatingMeasureMap(List<Pair<J, K>> data) {
			this();
			data.forEach(point -> put(point.getFirst(), point.getSecond()));
		}
	}

	/**
	 * The inverse of this transform "undoes" the effect of translating by this transform.
	 *
	 * @return The opposite of this transform.
	 */
	public static Pose2d inversePose(Pose2d pose) {
		Rotation2d rotation_inverted = inverseRotation(pose.getRotation());
		return new Pose2d(inverseTranslation(pose.getTranslation()).rotateBy(rotation_inverted), rotation_inverted);
	}

	/**
	 * The inverse of a Rotation2d "undoes" the effect of this rotation.
	 *
	 * @return The inverse of this rotation.
	 */
	public static Rotation2d inverseRotation(Rotation2d angle) {
		return new Rotation2d(angle.getCos(), -angle.getSin());
	}

	/**
	 * The inverse simply means a Translation2d that "undoes" this object.
	 *
	 * @return Translation by -x and -y.
	 */
	public static Translation2d inverseTranslation(Translation2d translation) {
		return new Translation2d(-translation.getX(), -translation.getY());
	}

	public static Rotation2d direction(Translation2d translation) {
		return new Rotation2d(translation.getX(), translation.getY());
	}

	public static double norm(Translation2d translation) {
		return Math.hypot(translation.getX(), translation.getY());
	}

	public static class Pose2dTimeInterpolable {
		private List<Pair<Pose2d, Time>> poseList = new ArrayList<>();

		public Time getTimeFromPose(Pose2d pose) {
			Pair<Pose2d, Time> prevState = poseList.get(0);
			Pair<Pose2d, Time> nextState = poseList.get(0);
			for (int i = 0; i < poseList.size() - 1; i++) {
				if (i >= poseList.size() - 2) {
					return poseList.get(poseList.size() - 1).getSecond();
				}
				prevState = poseList.get(i);
				nextState = poseList.get(i + 2);
				if (prevState.getFirst().getTranslation().getDistance(pose.getTranslation())
						< nextState.getFirst().getTranslation().getDistance(pose.getTranslation())) {
					nextState = poseList.get(i + 1);
					break;
				}
			}

			double distanceToPrevPose = prevState.getFirst().getTranslation().getDistance(pose.getTranslation());
			double distanceToNextPose = nextState.getFirst().getTranslation().getDistance(pose.getTranslation());
			double percentToNextPose = (prevState
									.getFirst()
									.getTranslation()
									.getDistance(nextState.getFirst().getTranslation())
							- distanceToNextPose)
					/ (distanceToPrevPose + distanceToNextPose);

			Time timeDelta = nextState.getSecond().minus(prevState.getSecond());
			Time timeAtPose = prevState.getSecond().plus(timeDelta.times(percentToNextPose));
			return timeAtPose;
		}

		public Pose2d getPoseFromTime(Time time) {
			if (time.gte(poseList.get(poseList.size() - 1).getSecond())) {
				return poseList.get(poseList.size() - 1).getFirst();
			} else if (time.lte(poseList.get(0).getSecond())) {
				return poseList.get(0).getFirst();
			}

			Pair<Pose2d, Time> prevState = poseList.get(0);
			Pair<Pose2d, Time> nextState = poseList.get(0);
			;

			for (int i = 1; i < poseList.size(); i++) {
				nextState = poseList.get(i);
				if (nextState.getSecond().gte(time)) {
					prevState = poseList.get(i - 1);
					break;
				}
			}

			Time timeDelta = nextState.getSecond().plus(prevState.getSecond());
			double percentIntoDelta =
					time.minus(prevState.getSecond()).in(BaseUnits.TimeUnit) / (timeDelta.in(BaseUnits.TimeUnit));
			Transform2d prevToTimePose =
					nextState.getFirst().minus(prevState.getFirst()).times(percentIntoDelta);
			return prevState.getFirst().plus(prevToTimePose);
		}

		public void clearStatesBeforeTime(Time time) {
			while (poseList.size() > 1 && poseList.get(0).getSecond().lt(time)) {
				poseList.remove(0);
			}
		}

		public Pose2dTimeInterpolable(Trajectory trajwithTan, Rotation2d startHeading, Rotation2d endHeading) {
			double totalTimeSecpnods = trajwithTan.getTotalTimeSeconds();
			for (State state : trajwithTan.getStates()) {
				Rotation2d poseRotation = startHeading.interpolate(endHeading, state.timeSeconds / totalTimeSecpnods);
				poseList.add(new Pair<>(
						new Pose2d(state.poseMeters.getTranslation(), poseRotation),
						Units.Seconds.of(state.timeSeconds)));
			}
			SmartDashboard.putNumber("Auto Align Traj/Number Of Trajectory States", poseList.size());
		}
	}

	/**
	 * Calculates the intersection points of two circles.
	 *
	 * @param center1 Center point of the first circle
	 * @param radius1 Radius of the first circle
	 * @param center2 Center point of the second circle
	 * @param radius2 Radius of the second circle
	 * @return An ArrayList containing all intersection points of the circle. ArrayList may have 0, 1, or 2 values.
	 */
	public static ArrayList<Translation2d> getCircleIntersectionPoints(
			Translation2d center1, double radius1, Translation2d center2, double radius2) {
		ArrayList<Translation2d> allPoints = new ArrayList<>();

		Translation2d delta = center2.minus(center1);
		double distance = center2.getDistance(center1);

		if (distance > (Math.abs(radius1) + Math.abs(radius2))) {
			return allPoints; // circles do not intersect
		}

		double a = (radius1 * radius1 - radius2 * radius2 + distance * distance) / (2 * distance);
		double h = Math.sqrt(radius1 * radius1 - a * a);

		Translation2d point0 = center1.plus((delta.times(a).div(distance)));

		Translation2d solution1 =
				point0.plus(new Translation2d(delta.getY(), delta.unaryMinus().getX())
						.times(h)
						.div(distance));
		Translation2d solution2 =
				point0.plus(new Translation2d(delta.unaryMinus().getY(), delta.getX())
						.times(h)
						.div(distance));

		allPoints.add(solution1);

		if (solution1 != solution2) {
			allPoints.add(solution2);
		}

		return allPoints;
	}

	public static Set<Subsystem> getEmptySubsystemSet() {
		Set<Subsystem> s = Set.of();
		return s;
	}

	public static Pose2d addPoses(Pose2d a, Pose2d b) {
		return new Pose2d(
				a.getTranslation().plus(b.getTranslation()), a.getRotation().plus(b.getRotation()));
	}

	public static Angle getLowestDelta(Angle base, Collection<Angle> options) {
		double lowestRotation = Double.MAX_VALUE;
		for (Angle a : options) {
			double diffRotations = (base.minus(a).abs(Units.Rotations));
			if (lowestRotation > diffRotations) {
				lowestRotation = diffRotations;
			}
		}
		return Units.Rotations.of(lowestRotation);
	}

	public static Command smartDashCommand(String message) {
		return Commands.defer(
				() -> Commands.runOnce(() -> SmartDashboard.putNumber(message, Timer.getFPGATimestamp())),
				getEmptySubsystemSet());
	}

	public static Distance getDistanceFromReef() {
		Pose2d flippedReefPose = FieldLayout.handleAllianceFlip(
				new Pose2d(FieldLayout.blueReefCenter, Rotation2d.kZero), RobotConstants.isRedAlliance);
		return Units.Meters.of(flippedReefPose
				.getTranslation()
				.getDistance(Drive.mInstance.getPose().getTranslation()));
	}

	public static Rotation2d getAngleFromReef() {
		Pose2d flippedReefPose = FieldLayout.handleAllianceFlip(
				new Pose2d(FieldLayout.blueReefCenter, Rotation2d.kZero), RobotConstants.isRedAlliance);
		Pose2d drivePose = Drive.mInstance.getPose();
		return drivePose
				.getTranslation()
				.minus(flippedReefPose.getTranslation())
				.getAngle()
				.minus(drivePose.getRotation());
	}

	public static <M extends Measure<U>, U extends Unit> M min(M x, M y) {
		return x.lt(y) ? x : y;
	}

	public static <M extends Measure<U>, U extends Unit> M max(M x, M y) {
		return x.gt(y) ? x : y;
	}

	public static class ScheduleIfWontCancelOther extends Command {
		private final Command command;

		public ScheduleIfWontCancelOther(Command commandToSchedule) {
			command = commandToSchedule;
		}

		@Override
		public void initialize() {
			for (Subsystem requirement : command.getRequirements()) {
				if (requirement.getCurrentCommand() != null) {
					return;
				}
			}
			command.schedule();
		}
	}

	public static double calculateDistanceToStartDeccel(
			double currentVel, double stowedAccel, double raisedAccel, double timeToRaise) {
		double distToRaiseElev = calculatDistanceToRaiseElevator(raisedAccel, timeToRaise);
		double velAtDistToRaiseElev = raisedAccel * timeToRaise;
		double deltaVel = currentVel - velAtDistToRaiseElev;
		double timeToDeccelDelta = deltaVel / stowedAccel;
		double distToDeccelDelta = 0.5 * stowedAccel * timeToDeccelDelta * timeToDeccelDelta;
		double totalDist = distToDeccelDelta + distToRaiseElev;
		return totalDist;
	}

	public static double calculatDistanceToRaiseElevator(double raisedAccel, double timeToRaise) {
		return 0.5 * raisedAccel * timeToRaise * timeToRaise;
	}
}
