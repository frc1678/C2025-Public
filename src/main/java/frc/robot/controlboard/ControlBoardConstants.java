package frc.robot.controlboard;

import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

public class ControlBoardConstants {
	public static final CommandXboxController mDriverController = new CommandXboxController(0);
	public static final CommandXboxController mOperatorController = new CommandXboxController(1);

	public static final Time kIntakeRumbleTime = Units.Seconds.of(0.2);

	public static final double stickDeadband = 0.05;
}
