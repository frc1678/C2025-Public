package frc.robot.subsystems.brakecoast;

import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.DigitalInput;
import frc.lib.io.MotorIO.Setpoint;
import frc.robot.Ports;
import frc.robot.subsystems.LEDs.LEDs;
import frc.robot.subsystems.algaedeploy.AlgaeDeploy;
import frc.robot.subsystems.climber.Climber;
import frc.robot.subsystems.coraldeploy.CoralDeploy;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.pivot.Pivot;

/**
 * Button on the robot which allows motors to be moved around easier by switching between brake and coast.
 */
public class BrakeCoastButton implements Sendable {

	private final DigitalInput input;
	private boolean lastState = false;
	private boolean isBrake = true;

	public BrakeCoastButton() {
		input = new DigitalInput(Ports.PHYSICAL_BUTTON.id);
	}

	@Override
	public void initSendable(SendableBuilder builder) {
		builder.addBooleanProperty("Brake Coast Button/LastState", () -> lastState, null);
		builder.addBooleanProperty("Brake Coast Button/Motors in Brake Mode", () -> isBrake, null);
	}

	public boolean isPressed() {
		boolean get = !input.get();
		boolean pressed = false;
		if (get && !lastState) {
			pressed = true;
		}
		lastState = get;
		return pressed;
	}

	public void toggleState() {
		isBrake = !isBrake;
		applyState();
	}

	public void applyState() {
		if (isBrake) {
			applyNeutral();
		} else {
			applyCoast();
		}

		LEDs.mInstance.setBrakeMode(isBrake);
	}

	public void applyNeutral() {
		AlgaeDeploy.mInstance.applySetpoint(Setpoint.withNeutralSetpoint());
		Climber.mInstance.applySetpoint(Setpoint.withNeutralSetpoint());
		CoralDeploy.mInstance.applySetpoint(Setpoint.withNeutralSetpoint());
		Elevator.mInstance.applySetpoint(Setpoint.withNeutralSetpoint());
		Pivot.mInstance.applySetpoint(Setpoint.withNeutralSetpoint());
	}

	public void applyCoast() {
		AlgaeDeploy.mInstance.applySetpoint(Setpoint.withCoastSetpoint());
		Climber.mInstance.applySetpoint(Setpoint.withCoastSetpoint());
		CoralDeploy.mInstance.applySetpoint(Setpoint.withCoastSetpoint());
		Elevator.mInstance.applySetpoint(Setpoint.withCoastSetpoint());
		Pivot.mInstance.applySetpoint(Setpoint.withCoastSetpoint());
	}

	public boolean isBrakeMode() {
		return isBrake;
	}
}
