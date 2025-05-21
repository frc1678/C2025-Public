package frc.lib.sim;

import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Unit;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.Timer;

/**
 * Abstract class for simulatable mechanisms.
 */
public abstract class MechanismSim {
	private Time lastTimeStamp;
	private final double gearing;

	/**
	 * Converts mechanism units to rotor units by accounting for gearing.
	 *
	 * @param mechansim Mechanism units.
	 * @return Rotor units.
	 */
	@SuppressWarnings("unchecked")
	public <T extends Measure<U>, U extends Unit> T mechanismToRotor(T mechanism) {
		return (T) mechanism.times(gearing);
	}

	/**
	 * Sets the motor voltage of the simulated mechanism.
	 *
	 * @param voltage Voltage to set.
	 */
	public abstract void setVoltage(Voltage voltage);

	/**
	 * Gets the mechansim position of the simulated system.
	 *
	 * @return Position of mechanism.
	 */
	public abstract Angle getPosition();

	/**
	 * Gets the mechansim velcocity of the simulated system.
	 *
	 * @return Velocity of mechanism.
	 */
	public abstract AngularVelocity getVelocity();

	/**
	 * Gets the stator current of the simulated system.
	 *
	 * @return Stator Current.
	 */
	public abstract Current getStatorCurrent();

	/**
	 * Updates the simulation based off the time since the last update.
	 *
	 * @param deltaTime The time since last update.
	 */
	protected abstract void update(Time deltaTime);

	/**
	 * Sets the simulation to be in a certain state.
	 *
	 * @param angle Angle to set.
	 * @param velocity Velocity to set.
	 */
	public abstract void setState(Angle angle, AngularVelocity velocity);

	/**
	 * Simulates the mechansim based on the change in time since the last time it was simulated.
	 */
	public void simulate() {
		Time currentTimestamp = Units.Seconds.of(Timer.getFPGATimestamp());
		update(currentTimestamp.minus(lastTimeStamp));
		lastTimeStamp = currentTimestamp;
	}

	/***
	 * Creates a MechanismSim
	 */
	protected MechanismSim(double gearing) {
		lastTimeStamp = Units.Seconds.of(Timer.getFPGATimestamp());
		this.gearing = gearing;
	}
}
