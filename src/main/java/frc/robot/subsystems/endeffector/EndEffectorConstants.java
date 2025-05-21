package frc.robot.subsystems.endeffector;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import frc.lib.io.MotorIOTalonFX;
import frc.lib.io.MotorIOTalonFX.MotorIOTalonFXConfig;
import frc.lib.io.MotorIOTalonFXSim;
import frc.lib.sim.RollerSim;
import frc.lib.sim.RollerSim.RollerSimConstants;
import frc.robot.Ports;
import frc.robot.Robot;

public class EndEffectorConstants {
	private static final double kCoralRollerGearing = (7.5 / 1.0);
	private static final double kAlgaeRollerGearing = (6.0 / 1.0);

	public static final Voltage kAlgaeHoldVoltage = Units.Volts.of(1.0);
	public static final Voltage kAlgaeReefIntakeVoltage = Units.Volts.of(12.0);
	public static final Voltage kAlgaeGroundIntakeVoltage = Units.Volts.of(12.0);
	public static final Voltage kNetAlgaeOuttakeVoltage = Units.Volts.of(-9.0);
	public static final Voltage kProcessorAlgaeOuttakeVoltage = Units.Volts.of(-3.0);
	public static final Voltage kSpitVoltage = Units.Volts.of(-3.0);

	public static final Voltage kCoralHoldVoltage = Units.Volts.of(1.0);
	public static final Voltage kCoralIntakeVoltage = Units.Volts.of(6.0);

	public static final Voltage kCoralOuttakeVoltageL1 = Units.Volts.of(-2.0);
	public static final Voltage kCoralOuttakeVoltageL2 = Units.Volts.of(-3.0);
	public static final Voltage kCoralOuttakeVoltageL3 = Units.Volts.of(-3.0);
	public static final Voltage kCoralOuttakeVoltageL4 = Units.Volts.of(-10.0);

	public static final Voltage kSoftCoralOuttakeVoltage = Units.Volts.of(-1.0);

	public static final Voltage kStationIntakeVoltage = Units.Volts.of(12.0);

	public static final Current kAlgaeStatorCurrentThreshold = Units.Amps.of(70.0);
	public static final Current kCoralStatorCurrentThreshold = Units.Amps.of(60.0);

	private static final TalonFXConfiguration getFXConfig() {
		TalonFXConfiguration config = new TalonFXConfiguration();

		config.Voltage.PeakForwardVoltage = 12.0;
		config.Voltage.PeakReverseVoltage = -12.0;

		config.CurrentLimits.StatorCurrentLimitEnable = Robot.isReal();
		config.CurrentLimits.StatorCurrentLimit = 80.0;

		config.CurrentLimits.SupplyCurrentLimitEnable = Robot.isReal();
		config.CurrentLimits.SupplyCurrentLimit = 60.0;
		config.CurrentLimits.SupplyCurrentLowerLimit = 60.0;
		config.CurrentLimits.SupplyCurrentLowerTime = 1.0;

		config.Feedback.SensorToMechanismRatio = kCoralRollerGearing;

		config.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
		config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
		return config;
	}

	public static final MotorIOTalonFXConfig getIOConfig() {
		MotorIOTalonFXConfig config = new MotorIOTalonFXConfig();
		config.mainBus = Ports.END_EFFECTOR.bus;
		config.mainID = Ports.END_EFFECTOR.id;
		config.mainConfig = getFXConfig();
		config.time = Units.Seconds;
		config.unit = Units.Degrees;
		return config;
	}

	public static final RollerSimConstants getSimConstants() {
		RollerSimConstants constants = new RollerSimConstants();
		constants.gearing = kCoralRollerGearing;
		constants.momentOfInertia = 0.000000001;
		constants.motor = new DCMotor(
				12, 4.05, 275, 1.4, edu.wpi.first.math.util.Units.rotationsPerMinuteToRadiansPerSecond(7530), 1);
		return constants;
	}

	public static final MotorIOTalonFX getMotorIO() {
		if (Robot.isReal()) {
			return new MotorIOTalonFX(getIOConfig());
		} else {
			return new MotorIOTalonFXSim(getIOConfig(), new RollerSim(getSimConstants()));
		}
	}
}
