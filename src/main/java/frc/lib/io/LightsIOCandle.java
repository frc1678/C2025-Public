package frc.lib.io;

import com.ctre.phoenix.led.CANdle;
import com.ctre.phoenix.led.CANdleConfiguration;
import frc.lib.io.LightsIO.State.RGBColor;

public class LightsIOCandle extends LightsIO {
	private final CANdle candle;

	public LightsIOCandle(LightsIOCandleConfiguration config) {
		super(config.ledCount);
		candle = new CANdle(config.id, config.bus);
		candle.configAllSettings(config.configuration);
	}

	protected void setLEDs(RGBColor color, int startIndex, int numLeds) {
		candle.setLEDs(color.r, color.g, color.b, 0, startIndex, numLeds);
	}

	public static class LightsIOCandleConfiguration {
		public int id;
		public String bus;
		public int ledCount;
		public CANdleConfiguration configuration = new CANdleConfiguration();
	}
}
