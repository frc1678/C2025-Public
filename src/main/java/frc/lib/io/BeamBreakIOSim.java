package frc.lib.io;

import edu.wpi.first.units.measure.Time;
import java.util.function.BooleanSupplier;

public class BeamBreakIOSim extends BeamBreakIO {
	private final BooleanSupplier button;

	public BeamBreakIOSim(BooleanSupplier buttonSupplier, Time debounce, String name) {
		super(debounce, name);
		button = buttonSupplier;
	}

	@Override
	public boolean get() {
		return button.getAsBoolean();
	}
}
