package frc.robot.subsystems.coralindexer;

import frc.lib.bases.MotorSubsystem;
import frc.lib.io.MotorIO.Setpoint;
import frc.lib.io.MotorIOTalonFX;

public class CoralIndexer extends MotorSubsystem<MotorIOTalonFX> {
	public static final Setpoint IDLE = Setpoint.withNeutralSetpoint();
	public static final Setpoint INTAKE = Setpoint.withVoltageSetpoint(CoralIndexerConstants.kIntakeVoltage);
	public static final Setpoint EXHAUST = Setpoint.withVoltageSetpoint(CoralIndexerConstants.kExhaustVoltage);

	public static final CoralIndexer mInstance = new CoralIndexer();

	private CoralIndexer() {
		super(CoralIndexerConstants.getMotorIO(), "Coral Indexer");
	}
}
