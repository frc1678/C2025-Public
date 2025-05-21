// Copyright (c) 2025 FRC 6328
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.lib.logging;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/** Utility class for logging code execution times. */
public class LoggedTracer {
	private LoggedTracer() {}

	private static double startTime = -1.0;

	/** Reset the clock. */
	public static void reset() {
		startTime = Timer.getFPGATimestamp();
	}

	/** Save the time elapsed since the last reset or record. */
	public static void record(String epochName) {
		double now = Timer.getFPGATimestamp();
		SmartDashboard.putNumber(
				"Logged Tracer/" + epochName + " Milliseconds", Units.secondsToMilliseconds(now - startTime));
		startTime = now;
	}
}
