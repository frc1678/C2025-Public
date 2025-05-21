package frc.lib.bases;

import edu.wpi.first.units.BaseUnits;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj2.command.Command;
import frc.lib.io.MotorIO;
import frc.lib.io.MotorIO.Setpoint;
import java.util.function.Supplier;

public class CoupledServoMotorSubsystem<IO extends MotorIO> extends ServoMotorSubsystem<IO> {
	private Supplier<Angle> coupledPositionSupplier;
	private Supplier<AngularVelocity> coupledVelocitySupplier;
	private double coupleGearing;

	public CoupledServoMotorSubsystem(
			IO io,
			String name,
			Angle epsilonThreshold,
			Supplier<Angle> coupledPositionSupplier,
			Supplier<AngularVelocity> coupledVelocitySupplier,
			double coupleGearing) {
		super(io, name, epsilonThreshold);
		this.coupledPositionSupplier = coupledPositionSupplier;
		this.coupledVelocitySupplier = coupledVelocitySupplier;
		this.coupleGearing = coupleGearing;
	}

	public CoupledServoMotorSubsystem(
			IO io,
			String name,
			Angle epsilonThreshold,
			ServoHomingConfig config,
			Supplier<Angle> coupledPositionSupplier,
			Supplier<AngularVelocity> coupledVelocitySupplier,
			double coupleGearing) {
		super(io, name, epsilonThreshold, config);
		this.coupledPositionSupplier = coupledPositionSupplier;
		this.coupledVelocitySupplier = coupledVelocitySupplier;
		this.coupleGearing = coupleGearing;
	}

	public Angle getCoupledPositionEffect() {
		try { // Handles if requisite subsystems for supplier are yet to be initialized
			return coupledPositionSupplier.get().times(coupleGearing);
		} catch (Exception e) {
			return BaseUnits.AngleUnit.of(0.0);
		}
	}

	public AngularVelocity getCoupledVelocityEffect() {
		try { // Handles if requisite subsystems for supplier are yet to be initialized
			return coupledVelocitySupplier.get().times(coupleGearing);
		} catch (Exception e) {
			return BaseUnits.AngleUnit.per(BaseUnits.TimeUnit).of(0.0);
		}
	}

	@Override
	public Command setpointCommand(Setpoint setpoint) {
		return followSetpointCommand(() -> setpoint);
	}

	@Override
	public void applySetpoint(Setpoint setpoint) {
		switch (setpoint.mode) {
			case MOTIONMAGIC:
				setpoint = Setpoint.withMotionMagicSetpoint(
						BaseUnits.AngleUnit.of(setpoint.baseUnits).plus(getCoupledPositionEffect()));
				break;
			case POSITIONPID:
				setpoint = Setpoint.withPositionSetpoint(
						BaseUnits.AngleUnit.of(setpoint.baseUnits).plus(getCoupledPositionEffect()));
				break;
			case VELOCITY:
				setpoint = Setpoint.withVelocitySetpoint(BaseUnits.AngleUnit.ofBaseUnits(setpoint.baseUnits)
						.per(BaseUnits.TimeUnit)
						.plus(getCoupledVelocityEffect()));
			default:
				break;
		}
		super.applySetpoint(setpoint);
	}

	@Override
	public void initSendable(SendableBuilder builder) {
		super.initSendable(builder);
		builder.addDoubleProperty(
				"Uncoupled Position " + io.unitType.name() + ":",
				() -> getPosition().in(io.unitType),
				null);
		builder.addDoubleProperty(
				"Uncoupled Velocity " + io.unitType.name() + " per " + io.time.name() + ":",
				() -> getVelocity().in(io.unitType.per(io.time)),
				null);
		builder.addDoubleProperty("Uncoupled Setpoint Value as Double:", () -> getSetpointDoubleInUnits(), null);
	}

	@Override
	public Angle getPosition() {
		return super.getPosition().minus(getCoupledPositionEffect());
	}

	@Override
	public AngularVelocity getVelocity() {
		return super.getVelocity().minus(getCoupledVelocityEffect());
	}

	@Override
	public void setCurrentPosition(Angle position) {
		super.setCurrentPosition(position.plus(getCoupledPositionEffect()));
	}

	@Override
	public double getSetpointDoubleInUnits() {
		synchronized (io) {
			if (getSetpoint().mode.isPositionControl()) {
				return BaseUnits.AngleUnit.of(getSetpoint().baseUnits)
						.minus(getCoupledPositionEffect())
						.in(io.unitType);
			} else if (getSetpoint().mode.isVelocityControl()) {
				return (BaseUnits.AngleUnit.of(getSetpoint().baseUnits).per(BaseUnits.TimeUnit))
						.minus(getCoupledVelocityEffect())
						.in(io.unitType.per(io.time));
			} else {
				return io.getSetpointDoubleInUnits();
			}
		}
	}
}
