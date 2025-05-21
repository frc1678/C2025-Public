package frc.robot.subsystems.coralrollers;

import frc.lib.bases.MotorSubsystem;
import frc.lib.io.MotorIO.Setpoint;
import frc.lib.io.MotorIOTalonFX;

public class CoralRollers extends MotorSubsystem<MotorIOTalonFX> {
	public static final Setpoint IDLE = Setpoint.withNeutralSetpoint();
	public static final Setpoint INTAKE = Setpoint.withVoltageSetpoint(CoralRollerConstants.kIntakeVoltage);
	public static final Setpoint EXHAUST = Setpoint.withVoltageSetpoint(CoralRollerConstants.kExhaustVoltage);
	public static final Setpoint PELICAN = Setpoint.withVoltageSetpoint(CoralRollerConstants.kPelicanVoltage);
	public static final Setpoint START = Setpoint.withVoltageSetpoint(CoralRollerConstants.kStartVoltage);

	public static final CoralRollers mInstance = new CoralRollers();

	public CoralRollers() {
		super(CoralRollerConstants.getMotorIO(), "Coral Rollers");
	}
}
