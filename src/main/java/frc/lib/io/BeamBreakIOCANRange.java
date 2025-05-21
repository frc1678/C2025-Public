package frc.lib.io;

import com.ctre.phoenix6.configs.CANrangeConfiguration;
import com.ctre.phoenix6.hardware.CANrange;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.Notifier;
import frc.robot.Ports;

public class BeamBreakIOCANRange extends BeamBreakIO {
	private final CANrange mBreak;
	private double lastSignalStrength = -1678;
	private final Notifier notifier;
	private final double signalStrengthThreshold;

	public static BeamBreakIOCANRange makeInverted(
			int channel,
			String canbus,
			CANrangeConfiguration config,
			Time debounce,
			String name,
			double signalStrengthThreshold) {
		return new BeamBreakIOCANRange(channel, canbus, config, debounce, name, signalStrengthThreshold) {
			@Override
			public boolean get() {
				return !super.get();
			}
		};
	}

	public static BeamBreakIOCANRange makeInverted(
			Ports port, CANrangeConfiguration config, Time debounce, String name, double signalStrengthThreshold) {
		return makeInverted(port.id, port.bus, config, debounce, name, signalStrengthThreshold);
	}

	public BeamBreakIOCANRange(
			int channel,
			String canbus,
			CANrangeConfiguration config,
			Time debounce,
			String name,
			double signalStrengthThreshold) {
		super(debounce, name);
		this.signalStrengthThreshold = signalStrengthThreshold;
		mBreak = new CANrange(channel, canbus);
		mBreak.getConfigurator().apply(config);
		notifier = new Notifier(() -> {
			lastSignalStrength = mBreak.getSignalStrength().getValueAsDouble();
		});

		notifier.startPeriodic(0.02);
	}

	public BeamBreakIOCANRange(
			Ports port, CANrangeConfiguration config, Time debounce, String name, double signalStrengthThreshold) {
		this(port.id, port.bus, config, debounce, name, signalStrengthThreshold);
	}

	@Override
	public boolean get() {
		return lastSignalStrength > signalStrengthThreshold;
	}

	@Override
	public void initSendable(SendableBuilder builder) {
		super.initSendable(builder);
		builder.addDoubleProperty("Signal Strength", () -> lastSignalStrength, null);
	}
}
