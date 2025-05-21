package frc.robot.subsystems.climberrollers;

import frc.lib.bases.MotorSubsystem;
import frc.lib.io.MotorIO.Setpoint;
import frc.lib.io.MotorIOTalonFX;

public class ClimberRollers extends MotorSubsystem<MotorIOTalonFX> {
	public static final Setpoint IDLE = Setpoint.withNeutralSetpoint();
	public static final Setpoint INTAKE = Setpoint.withVoltageSetpoint(ClimberRollerConstants.kIntakeVoltage);
	public static final Setpoint EXHAUST = Setpoint.withVoltageSetpoint(ClimberRollerConstants.kExhaustVoltage);

	public static final ClimberRollers mInstance = new ClimberRollers();

	private ClimberRollers() {
		super(ClimberRollerConstants.getMotorIO(), "Climber Rollers");
	}
}
