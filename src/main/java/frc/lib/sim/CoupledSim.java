package frc.lib.sim;

import edu.wpi.first.units.BaseUnits;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.units.measure.Voltage;
import java.util.function.Supplier;

/**
 * Class for simulating mechanically coupled systems.
 */
public class CoupledSim extends MechanismSim {
	private MechanismSim base;
	private final Supplier<Angle> coupledMechanismPositionSupplier;
	private final Supplier<AngularVelocity> coupledMechanismVelocitySupplier;
	private final double mechanismCoupleRatio;
	private Angle lastCoupledPositonEffect = BaseUnits.AngleUnit.of(0.0);
	private AngularVelocity lastCoupledVelocityEffect =
			BaseUnits.AngleUnit.of(0.0).per(BaseUnits.TimeUnit);

	private Angle getCoupledPositionEffect() {
		try { // Handles if requisite subsystems for supplier are yet to be initialized
			return coupledMechanismPositionSupplier.get().times(mechanismCoupleRatio);
		} catch (Exception e) {
			return BaseUnits.AngleUnit.of(0.0);
		}
	}

	private AngularVelocity getCoupledVelocityEffect() {
		try { // Handles if requisite subsystems for supplier are yet to be initialized
			return coupledMechanismVelocitySupplier.get().times(mechanismCoupleRatio);
		} catch (Exception e) {
			return BaseUnits.AngleUnit.of(0.0).per(BaseUnits.TimeUnit);
		}
	}

	@Override
	public void setVoltage(Voltage voltage) {
		base.setVoltage(voltage);
	}

	@Override
	public Angle getPosition() {
		return base.getPosition();
	}

	@Override
	public AngularVelocity getVelocity() {
		return base.getVelocity();
	}

	@Override
	public Current getStatorCurrent() {
		return base.getStatorCurrent();
	}

	@Override
	public void setState(Angle angle, AngularVelocity velocity) {
		base.setState(angle.plus(lastCoupledPositonEffect), velocity.plus(lastCoupledVelocityEffect));
	}

	@Override
	protected void update(Time deltaTime) {
		Angle currentCoupledPositionEffect = getCoupledPositionEffect();
		AngularVelocity currentCoupledVelocityEffect = getCoupledVelocityEffect();
		base.setState(
				base.getPosition().plus(currentCoupledPositionEffect.minus(lastCoupledPositonEffect)),
				base.getVelocity().plus(currentCoupledVelocityEffect.minus(lastCoupledVelocityEffect)));
		lastCoupledPositonEffect = currentCoupledPositionEffect;
		lastCoupledVelocityEffect = currentCoupledVelocityEffect;

		base.update(deltaTime);
	}

	public CoupledSim(
			MechanismSim baseSimulation,
			Supplier<Angle> coupledMechanismPositionSupplier,
			Supplier<AngularVelocity> coupledMechanismVelocitySupplier,
			double mechanismCoupleRatio) {
		super(baseSimulation.mechanismToRotor(BaseUnits.AngleUnit.of(1.0)).in(BaseUnits.AngleUnit));
		base = baseSimulation;
		this.coupledMechanismPositionSupplier = coupledMechanismPositionSupplier;
		this.coupledMechanismVelocitySupplier = coupledMechanismVelocitySupplier;
		this.mechanismCoupleRatio = mechanismCoupleRatio;
	}
}
