package frc.robot.subsystems.LEDs;

import com.ctre.phoenix.led.CANdle.LEDStripType;
import com.ctre.phoenix.led.CANdle.VBatOutputMode;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Time;
import frc.lib.io.LightsIO;
import frc.lib.io.LightsIO.State;
import frc.lib.io.LightsIO.State.RGBColor;
import frc.lib.io.LightsIOCandle;
import frc.lib.io.LightsIOCandle.LightsIOCandleConfiguration;
import frc.lib.io.LightsIOSim;
import frc.robot.Ports;
import frc.robot.Robot;
import frc.robot.RobotConstants;

public class LEDsConstants {
	public static final int candleLEDs = 8;
	public static final int frontLEDs = RobotConstants.isOmega ? 12 : 16;
	public static final int leftLEDs = 11;
	public static final int backLEDs = RobotConstants.isOmega ? 22 : 21;
	public static final int rightLEDs = RobotConstants.isOmega ? 22 : 21;

	public static final int stripLEDs = frontLEDs + leftLEDs + backLEDs + rightLEDs;

	public static final LightsIOCandleConfiguration getLightsIOCandleConfig() {
		LightsIOCandleConfiguration config = new LightsIOCandleConfiguration();
		config.id = Ports.CANDLE.id;
		config.bus = Ports.CANDLE.bus;
		config.ledCount = candleLEDs + stripLEDs;
		config.configuration.vBatOutputMode = VBatOutputMode.On;
		config.configuration.stripType = LEDStripType.GRB;
		return config;
	}

	public static final LightsIO getLightsIO() {
		if (Robot.isReal()) {
			return new LightsIOCandle(getLightsIOCandleConfig());
		} else {
			return new LightsIOSim(candleLEDs + stripLEDs);
		}
	}

	public static final RGBColor kL3AlgaeColor = new RGBColor(0, 255, 0);
	public static final RGBColor kL2AlgaeColor = new RGBColor(255, 0, 0);
	public static final RGBColor kLeftColorColor = new RGBColor(255, 0, 255);
	public static final RGBColor kRightFaceColor = new RGBColor(255, 255, 0);

	public static final LightsIO.Linear rainbow =
			LightsIO.Linear.getRainbow("Rainbow", Units.Seconds.of(1.0), 1, false);

	public static final State getLinearState(RGBColor color, boolean reverse) {
		return new LightsIO.Linear(
				"Linear " + color.toString(), Units.Seconds.of(1.0), 2, reverse, color, RGBColor.none);
	}

	public static final State getFlashingState(RGBColor color) {
		return new LightsIO.Flashing("Flashing " + color.toString(), Units.Seconds.of(0.25), color, RGBColor.none);
	}

	public static final State getFastFlashingState(RGBColor color) {
		return new LightsIO.Flashing("Fast Flashing " + color.toString(), Units.Seconds.of(0.1), RGBColor.none, color);
	}

	public static final State getSolidState(RGBColor color) {
		return new LightsIO.Solid("Solid " + color.toString(), color);
	}

	public static final Time kFastFlashDuration = Units.Seconds.of(0.6);
}
