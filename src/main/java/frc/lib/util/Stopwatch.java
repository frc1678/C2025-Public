package frc.lib.util;

import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.wpilibj.Timer;

public class Stopwatch {

	private double startTime = Double.POSITIVE_INFINITY;

	public void start() {
		startTime = Timer.getFPGATimestamp();
	}

	public void startIfNotRunning() {
		if (Double.isInfinite(startTime)) {
			start();
		}
	}

	public Time getTime() {
		if (Double.isInfinite(startTime)) {
			return Units.Seconds.of(0.0);
		}
		return Units.Seconds.of(Timer.getFPGATimestamp() - startTime);
	}

	public double getTimeAsDouble() {
		if (Double.isInfinite(startTime)) {
			return 0.0;
		}
		return Timer.getFPGATimestamp() - startTime;
	}

	public void reset() {
		startTime = Double.POSITIVE_INFINITY;
	}

	public void resetAndStart() {
		startTime = Double.POSITIVE_INFINITY;
		start();
	}
}
