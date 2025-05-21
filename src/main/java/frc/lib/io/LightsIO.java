package frc.lib.io;

import edu.wpi.first.math.Pair;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.Timer;
import frc.lib.io.LightsIO.State.RGBColor;
import java.util.ArrayList;

public abstract class LightsIO implements Sendable {
	protected ArrayList<Pair<State, Integer>> currentState = new ArrayList<>();
	private final int ledCount;

	public LightsIO(int ledCount) {
		this.ledCount = ledCount;
	}

	public ArrayList<Pair<State, Integer>> getCurrentState() {
		return currentState;
	}

	public String[] getStateNames() {
		String[] names = new String[currentState.size()];
		for (int i = 0; i < names.length; i++) {
			names[i] = currentState.get(i).getFirst().name;
		}
		return names;
	}

	@Override
	public void initSendable(SendableBuilder builder) {
		builder.addStringArrayProperty("State", () -> getStateNames(), null);
	}

	public void update() {
		int currentLEDIndex = 0;
		for (Pair<State, Integer> pair : currentState) {
			pair.getFirst().apply(this, currentLEDIndex, pair.getSecond());
			currentLEDIndex += pair.getSecond();
		}
	}

	public void setState(State state) {
		ArrayList<Pair<State, Integer>> list = new ArrayList<>();
		list.add(new Pair<State, Integer>(state, ledCount));
		setState(list);
	}

	public void setState(ArrayList<Pair<State, Integer>> state) {
		currentState = state;
	}

	protected abstract void setLEDs(RGBColor color, int startIndex, int numLeds);

	public abstract static class State {
		public final String name;

		private State(String name) {
			this.name = name;
		}

		public abstract void apply(LightsIO io, int startIndex, int numLeds);

		public static class RGBColor {
			public final int r;
			public final int g;
			public final int b;

			public RGBColor(int red, int green, int blue) {
				r = red;
				g = green;
				b = blue;
			}

			public static RGBColor fromWPIColor(edu.wpi.first.wpilibj.util.Color color) {
				return new RGBColor(
						Double.valueOf(color.red * 255).intValue(),
						Double.valueOf(color.green * 255).intValue(),
						Double.valueOf(color.blue * 255).intValue());
			}

			public static final RGBColor lime = new RGBColor(102, 255, 88);
			public static final RGBColor none = new RGBColor(0, 0, 0);
			public static final RGBColor red = new RGBColor(255, 0, 0);
			public static final RGBColor orange = new RGBColor(255, 185, 0);
			public static final RGBColor yellow = new RGBColor(255, 255, 0);
			public static final RGBColor green = new RGBColor(0, 255, 0);
			public static final RGBColor blue = new RGBColor(0, 0, 255);
			public static final RGBColor aqua = new RGBColor(0, 255, 255);
			public static final RGBColor purple = new RGBColor(255, 0, 255);

			public String toString() {
				return "R: " + r + ", G:" + g + ", B:" + b;
			}
		}
	}

	public static class Solid extends State {
		public final RGBColor color;

		public Solid(String name, RGBColor color) {
			super(name);
			this.color = color;
		}

		@Override
		public void apply(LightsIO io, int startIndex, int numLeds) {
			io.setLEDs(color, startIndex, numLeds);
		}
	}

	public static class Flashing extends State {
		public final RGBColor[] colors;
		public final Time interval;
		public Time lastUpdateTime = Units.Seconds.of(0.0);
		public int currentIndex = 0;

		public Flashing(String name, Time interval, RGBColor... colors) {
			super(name);
			this.interval = interval;
			this.colors = colors;
		}

		@Override
		public void apply(LightsIO io, int startIndex, int numLeds) {
			Time currentTime = Units.Seconds.of(Timer.getFPGATimestamp());

			if (currentTime.minus(lastUpdateTime).gte(interval)) {
				if (currentTime.minus(lastUpdateTime).gte(interval.times(2.0))) {
					currentIndex = 0;
				} else {
					currentIndex++;
					if (currentIndex >= colors.length) {
						currentIndex = 0;
					}
				}
				lastUpdateTime = currentTime;
			}
			io.setLEDs(colors[currentIndex], startIndex, numLeds);
		}
	}

	public static class Linear extends State {
		public final RGBColor[] colors;
		public final Time interval;
		public final boolean reverse;
		public Time lastUpdateTime = Units.Seconds.of(0.0);

		public Linear(String name, Time interval, RGBColor... colors) {
			this(name, interval, 1, false, colors);
		}

		public Linear(String name, Time interval, int numLines, boolean reverse, RGBColor... colors) {
			super(name);
			this.interval = interval;
			this.colors = new RGBColor[colors.length * numLines];
			this.reverse = reverse;
			for (int i = 0; i < numLines; i++) {
				for (int j = 0; j < colors.length; j++) {
					this.colors[j + i * colors.length] = colors[j];
				}
			}
		}

		public static final Linear getRainbow(String name, Time interval, int numLines, boolean reverse) {
			return new Linear(
					name,
					interval,
					numLines,
					reverse,
					RGBColor.red,
					RGBColor.orange,
					RGBColor.yellow,
					RGBColor.green,
					RGBColor.blue,
					RGBColor.purple);
		}

		@Override
		public void apply(LightsIO io, int startIndex, int numLeds) {
			double percentIntoInterval =
					(Timer.getFPGATimestamp() % interval.in(Units.Seconds)) / interval.in(Units.Seconds);
			int ledsPerColor = numLeds / colors.length;
			int offset = reverse
					? startIndex
							+ numLeds
							- Double.valueOf(percentIntoInterval * numLeds).intValue()
					: Double.valueOf(percentIntoInterval * numLeds).intValue();

			for (int i = 0; i < colors.length; i++) {
				int beginIndex = (offset + (ledsPerColor * i)) % numLeds + startIndex;
				int overlap = (beginIndex + ledsPerColor) - (startIndex + numLeds);
				if (overlap > 0) { // if it'll overlflow past the end
					io.setLEDs(colors[i], beginIndex, ledsPerColor - overlap);
					io.setLEDs(colors[i], startIndex, overlap);
				} else {
					io.setLEDs(colors[i], beginIndex, ledsPerColor);
					;
				}
			}
		}
	}

	public static class MultiState extends State {
		public final State[] states;

		public MultiState(State... states) {
			super(generateName(states));
			this.states = states;
		}

		public static String generateName(State[] states) {
			String name = states[0].name;
			for (int i = 1; i < states.length; i++) {
				name += ", " + states[i].name;
			}
			return name;
		}

		@Override
		public void apply(LightsIO io, int startIndex, int numLeds) {
			int ledsPerSection = numLeds / states.length;
			for (int i = 0; i < states.length; i++) {
				states[i].apply(io, startIndex + (ledsPerSection * (i)), ledsPerSection);
			}
		}
	}
}
