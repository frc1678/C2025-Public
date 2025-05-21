package frc.lib.io;

import edu.wpi.first.units.measure.Time;
import edu.wpi.first.wpilibj.DigitalInput;

public class BeamBreakIODigitalIn extends BeamBreakIO {
	private final DigitalInput mBreak;

	public static BeamBreakIODigitalIn makeInverted(int channel, Time debounce, String name) {
		return new BeamBreakIODigitalIn(channel, debounce, name) {
			@Override
			public boolean get() {
				return !super.get();
			}
		};
	}

	public BeamBreakIODigitalIn(int channel, Time debounce, String name) {
		super(debounce, name);
		mBreak = new DigitalInput(channel);
	}

	@Override
	public boolean get() {
		return mBreak.get();
	}
}
