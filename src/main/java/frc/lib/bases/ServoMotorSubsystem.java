package frc.lib.bases;

import edu.wpi.first.units.BaseUnits;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.*;
import frc.lib.io.MotorIO;
import frc.lib.io.MotorIO.Setpoint;
import frc.lib.util.DelayedBoolean;
import frc.lib.util.Util;

/**
 * Base subsystem for any subsystem that uses motors and requires precise position control.
 */
public class ServoMotorSubsystem<IO extends MotorIO> extends MotorSubsystem<IO> {
	protected final Angle epsilonThreshold;

	// Homing-specific variables
	protected boolean isHomingSubsystem;
	private ServoHomingConfig homingConfig;
	private boolean mHoming = false;
	private boolean mNeedsToHome = true;
	private DelayedBoolean mHomingDelay;

	/**
	 * Creates a *non-homing* ServoMotorSubsystem with a MotorIO, name for telemetry, and threshold for differences in measurement.
	 *
	 * @param io MotorIO for the subsystem.
	 * @param name Name for telemetry.
	 * @param epsilonThreshold Acceptable error range for position.
	 */
	public ServoMotorSubsystem(IO io, String name, Angle epsilonThreshold) {
		super(io, name);
		this.isHomingSubsystem = false;
		this.epsilonThreshold = epsilonThreshold;
	}

	/**
	 * Creates a *homing* ServoMotorSubsystem with a MotorIO, name for telemetry, and threshold for differences in measurement.
	 *
	 * @param io MotorIO for the subsystem.
	 * @param name Name for telemetry.
	 * @param epsilonThreshold Acceptable error range for position.
	 * @param config Homing configuration.
	 */
	public ServoMotorSubsystem(IO io, String name, Angle epsilonThreshold, ServoHomingConfig config) {
		this(io, name, epsilonThreshold);
		this.isHomingSubsystem = true;
		homingConfig = config;
		mHomingDelay = new DelayedBoolean(Timer.getFPGATimestamp(), homingConfig.kHomingTimeout.in(Units.Seconds));
	}

	@Override
	public void periodic() {
		super.periodic();
		if (isHomingSubsystem) {
			if (mNeedsToHome && setpointNearHome() && nearHomingLocation()) {
				mHoming = true;
				useSoftLimits(false);
				mHomingDelay =
						new DelayedBoolean(Timer.getFPGATimestamp(), homingConfig.kHomingTimeout.in(Units.Seconds));
			}
			if (mHoming) {
				io.applySetpoint(Setpoint.withVoltageSetpoint(homingConfig.kHomingVoltage));
				if (mHomingDelay.update(
						Timer.getFPGATimestamp(),
						Math.abs(getVelocity().baseUnitMagnitude()) < homingConfig.kSetHomedVelocity.baseUnitMagnitude()
								&& DriverStation.isEnabled())) {
					setCurrentPosition(homingConfig.kHomePosition);
					applySetpoint(Setpoint.withMotionMagicSetpoint(homingConfig.kHomePosition));
					useSoftLimits(true);
					mNeedsToHome = false;
				}
			}
		}
	}

	/**
	 * Determines whether the currently set setpoint is near subsystem's home position.
	 *
	 * @return True if setpoint is near the home position, false if not.
	 */
	public boolean setpointNearHome() {
		return getSetpoint().mode.isPositionControl()
				&& Util.epsilonEquals(
						getSetpointDoubleInUnits(),
						homingConfig.kHomePosition.in(io.unitType),
						epsilonThreshold.in(io.unitType));
	}

	/**
	 * Determines whether the subsystem is near it's homing position.
	 *
	 * @return True if currently near home position, false if not.
	 */
	public boolean nearHomingLocation() {
		return nearPosition(homingConfig.kHomePosition);
	}

	/**
	 * Determines whether the subsystem is near it's position setpoint.
	 *
	 * @return True if currently near setpoint, false if not. Returns false if not in position control.
	 */
	public boolean nearPositionSetpoint() {
		return (getSetpoint().mode.isPositionControl()) && nearPosition(getPosition());
	}

	/**
	 * Gets the current setpoint value of the MotorIO using units of the MotorIO.
	 *
	 * @return Setpoint in mechanism units.
	 */
	public double getSetpointDoubleInUnits() {
		return io.getSetpointDoubleInUnits();
	}

	/**
	 * Determines whether the subsystem is near a given position.
	 *
	 * @param mechanismPosition Position to compare to.
	 * @return True if near provided position, false if not.
	 */
	public boolean nearPosition(Angle mechanismPosition) {
		return Util.epsilonEquals(
				getPosition().in(BaseUnits.AngleUnit),
				mechanismPosition.in(BaseUnits.AngleUnit),
				epsilonThreshold.in(BaseUnits.AngleUnit));
	}

	@Override
	public void applySetpoint(Setpoint setpoint) {
		super.applySetpoint(setpoint);
		if (isHomingSubsystem) {
			if (mHoming) {
				mHoming = false;
				useSoftLimits(true);
			}
			if (!setpointNearHome()) {
				mNeedsToHome = true;
			}
		}
	}

	/**
	 * Creates a Command that waits until the mechanism is near a given position.
	 *
	 * @param mechanismPosition Position to evaluate proximity to.
	 * @return A wait command.
	 */
	public Command waitForPositionCommand(Angle mechanismPosition) {
		return Commands.waitUntil(() -> {
			return nearPosition(mechanismPosition);
		});
	}

	/**
	 * Creates a Command that goes to a setpoint and then waits until the mechanism is the setpoint's position.
	 *
	 * @param mechanismPosition Position to evaluate proximity to.
	 * @return A new Command to apply setpoint and wait.
	 */
	public Command setpointCommandWithWait(Setpoint setpoint) {
		return waitForPositionCommand(BaseUnits.AngleUnit.of(setpoint.baseUnits))
				.deadlineFor(setpointCommand(setpoint));
	}

	/**
	 * Enables or disables soft limits.
	 *
	 * @param enable True to enable, soft to disable.
	 */
	public void useSoftLimits(boolean enable) {
		io.useSoftLimits(enable);
	}

	/**
	 * Sets neutral mode to brake or coast.
	 *
	 * @param wantsBrake True for brake, false for coast.
	 */
	public void setNeutralBrake(boolean wantsBrake) {
		io.setNeutralBrake(wantsBrake);
	}

	/**
	 * Set's the mechanism's current location as a given position.
	 *
	 * @param mechanismPosition the mechanism's position to set location as.
	 */
	public void setCurrentPosition(Angle position) {
		io.setCurrentPosition(position);
	}

	@Override
	public void initSendable(SendableBuilder builder) {
		super.initSendable(builder);
		io.initSendable(builder);
		if (isHomingSubsystem) {
			builder.addBooleanProperty("Homing", () -> mHoming, null);
			builder.addBooleanProperty("Needs to Home", () -> mNeedsToHome, null);
		}
	}

	/**
	 * Configuration to make a homing ServoMotorSubsystem
	 */
	public static class ServoHomingConfig {
		public Angle kHomePosition;
		public Voltage kHomingVoltage;
		public Time kHomingTimeout;
		public AngularVelocity kSetHomedVelocity;
	}
}
