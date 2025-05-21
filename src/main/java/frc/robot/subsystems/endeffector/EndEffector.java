package frc.robot.subsystems.endeffector;

import frc.lib.bases.MotorSubsystem;
import frc.lib.io.MotorIO.Setpoint;
import frc.lib.io.MotorIOTalonFX;
import frc.lib.util.FieldLayout.Level;

public class EndEffector extends MotorSubsystem<MotorIOTalonFX> {
	public static final Setpoint IDLE = Setpoint.withCoastSetpoint();
	public static final Setpoint ALGAE_INTAKE =
			Setpoint.withVoltageSetpoint(EndEffectorConstants.kAlgaeReefIntakeVoltage);
	public static final Setpoint ALGAE_FEED =
			Setpoint.withVoltageSetpoint(EndEffectorConstants.kAlgaeGroundIntakeVoltage);
	public static final Setpoint ALGAE_HOLD = Setpoint.withVoltageSetpoint(EndEffectorConstants.kAlgaeHoldVoltage);

	public static final Setpoint SPIT = Setpoint.withVoltageSetpoint(EndEffectorConstants.kSpitVoltage);

	public static final Setpoint PROCESSOR_ALGAE_SCORE =
			Setpoint.withVoltageSetpoint(EndEffectorConstants.kProcessorAlgaeOuttakeVoltage);
	public static final Setpoint NET_ALGAE_SCORE =
			Setpoint.withVoltageSetpoint(EndEffectorConstants.kNetAlgaeOuttakeVoltage);

	public static final Setpoint CORAL_FEED = Setpoint.withVoltageSetpoint(EndEffectorConstants.kCoralIntakeVoltage);
	public static final Setpoint CORAL_HOLD = Setpoint.withVoltageSetpoint(EndEffectorConstants.kCoralHoldVoltage);

	public static final Setpoint CORAL_SCORE_L1 =
			Setpoint.withVoltageSetpoint(EndEffectorConstants.kCoralOuttakeVoltageL1);
	public static final Setpoint CORAL_SCORE_L2 =
			Setpoint.withVoltageSetpoint(EndEffectorConstants.kCoralOuttakeVoltageL2);
	public static final Setpoint CORAL_SCORE_L3 =
			Setpoint.withVoltageSetpoint(EndEffectorConstants.kCoralOuttakeVoltageL3);
	public static final Setpoint CORAL_SCORE_L4 =
			Setpoint.withVoltageSetpoint(EndEffectorConstants.kCoralOuttakeVoltageL4);

	public static final Setpoint SOFT_CORAL_SCORE =
			Setpoint.withVoltageSetpoint(EndEffectorConstants.kSoftCoralOuttakeVoltage);

	public static final Setpoint STATION_INTAKE =
			Setpoint.withVoltageSetpoint(EndEffectorConstants.kStationIntakeVoltage);

	public static final EndEffector mInstance = new EndEffector();

	private EndEffector() {
		super(EndEffectorConstants.getMotorIO(), "End Effector");
	}

	public static Setpoint getCoralScoreSetpoint(Level level) {
		return switch (level) {
			case L4 -> CORAL_SCORE_L4;
			case L3 -> CORAL_SCORE_L3;
			case L2 -> CORAL_SCORE_L2;
			case L1 -> CORAL_SCORE_L1;
			default -> CORAL_SCORE_L4;
		};
	}
}
