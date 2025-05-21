package frc.lib.bases;

import edu.wpi.first.units.BaseUnits;
import edu.wpi.first.units.measure.AngularVelocity;
import frc.lib.io.MotorIO;
import frc.lib.util.Util;

/**
 * Base subsystem for any subsystem that uses motors and requires precise velocity control.
 */
public class FlywheelMotorSubsystem<IO extends MotorIO> extends MotorSubsystem<IO> {
	protected final AngularVelocity epsilonThreshold;

	/**
	 * Creates a FlywheelMotorSubsystem with a MotorIO, name for telemetry, and threshold for differences in measurement.
	 *
	 * @param io MotorIO for the subsystem.
	 * @param name Name for telemetry.
	 * @param epsilonThreshold Acceptable error range for velocity.
	 */
	public FlywheelMotorSubsystem(IO io, String name, AngularVelocity epsilonThreshold) {
		super(io, name);
		this.epsilonThreshold = epsilonThreshold;
	}

	/**
	 * Gets whether or not the subsystem is within an acceptable threshold of a provided velocity.
	 *
	 * @param velocity Velocity to check proximity to.
	 * @return Whether the subsystem is acceptably near the given velocity.
	 */
	public boolean nearVelocity(AngularVelocity velocity) {
		return Util.epsilonEquals(
				velocity.baseUnitMagnitude(), getVelocity().baseUnitMagnitude(), epsilonThreshold.baseUnitMagnitude());
	}

	/**
	 * Gets whether or not the subsystem is within an acceptable threshold of it's velocity setpoint.
	 *
	 * @return Whether the subsystem is acceptably near it's setpoint's velocity. Returns false if not in velcity coontrol mode.
	 */
	public boolean spunUp() {
		return nearVelocity(BaseUnits.AngleUnit.per(BaseUnits.TimeUnit).of(io.getSetpoint().baseUnits))
				&& io.getSetpoint().mode.isVelocityControl();
	}
}
