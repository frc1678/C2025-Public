package frc.robot.subsystems.LEDs;

import edu.wpi.first.math.Pair;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.lib.bases.LightsSubsystem;
import frc.lib.io.LightsIO;
import frc.lib.io.LightsIO.State;
import frc.lib.io.LightsIO.State.RGBColor;
import frc.robot.controlboard.ControlBoard;
import frc.robot.subsystems.pivot.Pivot;
import frc.robot.subsystems.pivot.PivotConstants;
import frc.robot.subsystems.superstructure.Superstructure;
import frc.robot.subsystems.vision.Limelight;
import java.util.ArrayList;

public class LEDs extends LightsSubsystem<LightsIO> {
	private boolean isBrakeMode = true;
	public Time lastFlashStart = Units.Seconds.of(0.0);

	public static final LEDs mInstance = new LEDs();

	public LEDs() {
		super("LEDs", LEDsConstants.getLightsIO());
	}

	public State getDataState() {
		RGBColor color = getCurrentColor();
		return getShouldFlash() ? LEDsConstants.getFastFlashingState(color) : LEDsConstants.getSolidState(color);
	}

	public boolean getShouldFlash() {
		return Units.Seconds.of(Timer.getFPGATimestamp()).lte(lastFlashStart.plus(LEDsConstants.kFastFlashDuration));
	}

	public State getBrakeState() {
		if (isBrakeMode) {
			return LEDsConstants.getSolidState(RGBColor.green);
		} else {
			return LEDsConstants.getFlashingState(RGBColor.aqua);
		}
	}

	public ArrayList<Pair<State, Integer>> getStateToApply() {
		ArrayList<Pair<State, Integer>> list = new ArrayList<>();
		list.add(new Pair<State, Integer>(LEDsConstants.rainbow, LEDsConstants.candleLEDs));

		if (DriverStation.isEnabled()) {
			list.add(new Pair<State, Integer>(getDataState(), LEDsConstants.stripLEDs));
		} else {
			State dataState = getDataState();
			State brakeState = getBrakeState();
			list.add(new Pair<State, Integer>(dataState, LEDsConstants.frontLEDs));
			list.add(new Pair<State, Integer>(brakeState, LEDsConstants.leftLEDs));
			list.add(new Pair<State, Integer>(getVisionLEDsState(), LEDsConstants.backLEDs));
			list.add(new Pair<State, Integer>(getPivotZeroLEDsState(), LEDsConstants.rightLEDs));
		}
		return list;
	}

	public State getPivotZeroLEDsState() {
		Angle position = Pivot.mInstance.getAbsolutePosition();
		if (position.minus(PivotConstants.kAutoStart).abs(Units.Degrees) < 2.0) {
			return LEDsConstants.getSolidState(RGBColor.green);
		} else {
			int amountOff;
			try {
				amountOff = ((int) (Pivot.mInstance
						.getAbsolutePosition()
						.minus(PivotConstants.kAutoStart)
						.in(Units.Degrees)));
			} catch (Exception e) {
				amountOff = 0;
			}

			int ledColor = Math.min(Math.abs(amountOff), 255);
			return LEDsConstants.getLinearState(new RGBColor(ledColor, 255 - ledColor, 0), (amountOff < 0));
		}
	}

	public State getVisionLEDsState() {
		if (Limelight.mInstance.getPoseStable()) {
			return LEDsConstants.getSolidState(RGBColor.green);
		} else {
			return LEDsConstants.getFlashingState(RGBColor.yellow);
		}
	}

	public void setBrakeMode(boolean isBrake) {
		isBrakeMode = isBrake;
	}

	public RGBColor getCurrentColor() {
		if (ControlBoard.mInstance.getCoralMode()) {
			return getLeftRightColor(
					Superstructure.mInstance.getTargetingBranch().getKey().isLeft());
		} else {
			return getAlgaeL2L3Color(Superstructure.mInstance.getTargetingL3Algae());
		}
	}

	public RGBColor getLeftRightColor(boolean isLeft) {
		return isLeft ? LEDsConstants.kLeftColorColor : LEDsConstants.kRightFaceColor;
	}

	public RGBColor getAlgaeL2L3Color(boolean isL3) {
		return isL3 ? LEDsConstants.kL3AlgaeColor : LEDsConstants.kL2AlgaeColor;
	}

	@Override
	public void periodic() {
		ArrayList<Pair<State, Integer>> stateToApply = getStateToApply();
		ArrayList<Pair<State, Integer>> currentState = io.getCurrentState();
		boolean alreadyInState = areSameStates(stateToApply, currentState);
		if (!alreadyInState) {
			io.setState(stateToApply);
		}
		super.periodic();
	}

	public boolean areSameStates(ArrayList<Pair<State, Integer>> a, ArrayList<Pair<State, Integer>> b) {
		if (a.size() != b.size()) {
			return false;
		} else {
			for (int i = 0; i < a.size(); i++) {
				Pair<State, Integer> ai = a.get(i);
				Pair<State, Integer> bi = b.get(i);
				if ((!ai.getFirst().name.equals(bi.getFirst().name)) || (ai.getSecond() != bi.getSecond())) {
					return false;
				}
			}
		}
		return true;
	}

	public void startFlash() {
		lastFlashStart = Units.Seconds.of(Timer.getFPGATimestamp());
	}

	public Command flashCommand() {
		return runOnce(() -> startFlash());
	}
}
