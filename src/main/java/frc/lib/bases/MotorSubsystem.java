package frc.lib.bases;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.io.MotorIO;
import frc.lib.io.MotorIO.Setpoint;
import frc.lib.logging.LoggedTracer;
import java.util.function.Supplier;

/**
 * Base subsystem for any subsystem that uses motors.
 */
public class MotorSubsystem<IO extends MotorIO> extends SubsystemBase {
	protected final IO io;
	protected final String name;

	/**
	 * Creates a MotorSubsystem with a MotorIO and name for telemetry.
	 * @param io MotorIO for the subsystem.
	 * @param name Name for telemetry.
	 */
	public MotorSubsystem(IO io, String name) {
		super(name);
		this.io = io;
		this.name = name;
	}

	@Override
	public void periodic() {
		io.updateInputs();
		outputTelemetry();
	}

	/**
	 * Outputs subsystem readings and to SmartDashboard.
	 */
	public void outputTelemetry() {
		LoggedTracer.record(name);
	}

	/**
	 * Initializes this Sendable object. Override this should you need to output more information to SmartDashboard.
	 *
	 * @param builder Sendable builder.
	 */
	@Override
	public void initSendable(SendableBuilder builder) {
		super.initSendable(builder);
		io.initSendable(builder);
	}

	/**
	 * Gets the last read position of the subsystem's main motor.
	 *
	 * @return Position of the subsystem.
	 */
	public Angle getPosition() {
		return io.getPosition();
	}

	/**
	 * Gets the last read velocity of the subsystem's main motor.
	 *
	 * @return Velocity of the subsystem.
	 */
	public AngularVelocity getVelocity() {
		return io.getVelocity();
	}

	/**
	 * Gets the last read stator current of the subsystem's main motor.
	 *
	 * @return Stator current of the subsystem.
	 */
	public Current getStatorCurrent() {
		return io.getStatorCurrent();
	}

	/**
	 * Gets the last read supply current of the subsystem's main motor.
	 *
	 * @return Supply current of the subsystem.
	 */
	public Current getSupplyCurrent() {
		return io.getSupplyCurrent();
	}

	/**
	 * Gets the last read output voltage of the subsystem's main motor.
	 *
	 * @return Output voltage of the subsystem.
	 */
	public Voltage getMotorVoltage() {
		return io.getMotorVoltage();
	}

	/**
	 * Gets the last applied setpoint to the MotorIO.
	 *
	 * @return Last applied Setpoint.
	 */
	public Setpoint getSetpoint() {
		return io.getSetpoint();
	}

	/**
	 * Applies a Setpoint to the MotorIO.
	 *
	 * @param setpoint Setpoint to apply.
	 */
	public void applySetpoint(Setpoint setpoint) {
		io.applySetpoint(setpoint);
	}

	/**
	 * Creates a one time, instantaneus command for the subsystem to go to a given Setpoint.
	 *
	 * @param setpoint Setpoint to go to.
	 * @return One time Command for the subsystem.
	 */
	public Command setpointCommand(Setpoint setpoint) {
		return runOnce(() -> applySetpoint(setpoint));
	}

	/**
	 * Creates a continous command for the subsystem to repeatedly go to a supplied setpoint.
	 *
	 * @param setpoint Supplier of setpoint to go to.
	 * @return Continuous Command for the subsystem.
	 */
	public Command followSetpointCommand(Supplier<Setpoint> supplier) {
		return run(() -> applySetpoint(supplier.get()));
	}

	/**
	 * Disabled this Subsystem's MotorIO. Setpoints can still be set when disabled but will not be applied until re-enabled.
	 */
	public void disable() {
		io.disable();
	}

	/**
	 * Enables this Subsystem's MotorIO. Immediatly applies the last set setpoint including setpoints set when disabled. MotorIO is enabled by default.
	 */
	public void enable() {
		io.enable();
	}

	/**
	 * Creates a command to disable this Subsystem's MotorIO.
	 *
	 * @return An instantaneus Command not requiring this Subsystem.
	 */
	public Command disableCommand() {
		return Commands.runOnce(() -> io.disable());
	}

	/**
	 * Creates a command to enable this Subsystem's MotorIO.
	 *
	 * @return An instantaneus Command not requiring this Subsystem.
	 */
	public Command enableCommand() {
		return Commands.runOnce(() -> io.enable());
	}
}
