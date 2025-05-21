package frc.robot.subsystems.coralindexer;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Voltage;
import frc.lib.io.MotorIOTalonFX;
import frc.lib.io.MotorIOTalonFX.MotorIOTalonFXConfig;
import frc.lib.io.MotorIOTalonFXSim;
import frc.lib.sim.RollerSim;
import frc.lib.sim.RollerSim.RollerSimConstants;
import frc.robot.Ports;
import frc.robot.Robot;

public class CoralIndexerConstants {

	public static final Voltage kIntakeVoltage = Units.Volts.of(10.0);
	public static final Voltage kExhaustVoltage = Units.Volts.of(-8.0);
	public static final double kGearing = 2.5;

	public static TalonFXConfiguration getFXConfig() {
		TalonFXConfiguration FXConfig = new TalonFXConfiguration();

		FXConfig.CurrentLimits.StatorCurrentLimitEnable = Robot.isReal();
		FXConfig.CurrentLimits.StatorCurrentLimit = 120.0;

		FXConfig.CurrentLimits.SupplyCurrentLimitEnable = Robot.isReal();
		FXConfig.CurrentLimits.SupplyCurrentLimit = 60.0;
		FXConfig.CurrentLimits.SupplyCurrentLowerLimit = 60.0;
		FXConfig.CurrentLimits.SupplyCurrentLowerTime = 0.1;

		FXConfig.Voltage.PeakForwardVoltage = 12.0;
		FXConfig.Voltage.PeakReverseVoltage = -12.0;

		FXConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;

		FXConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
		FXConfig.Feedback.SensorToMechanismRatio = kGearing;

		return FXConfig;
	}

	public static MotorIOTalonFXConfig getIOConfig() {
		MotorIOTalonFXConfig config = new MotorIOTalonFXConfig();
		config.mainConfig = getFXConfig();
		config.mainID = Ports.CORAL_INDEXER.id;
		config.mainBus = Ports.CORAL_INDEXER.bus;
		config.time = Units.Second;
		config.unit = Units.Degree;
		return config;
	}

	public static RollerSimConstants getSimConstants() {
		RollerSimConstants simConstants = new RollerSimConstants();

		simConstants.gearing = kGearing;
		simConstants.motor = DCMotor.getKrakenX60Foc(1);
		simConstants.momentOfInertia = 0.0000002;

		return simConstants;
	}

	public static MotorIOTalonFX getMotorIO() {
		if (Robot.isReal()) {
			return new MotorIOTalonFX(getIOConfig());
		} else {
			return new MotorIOTalonFXSim(getIOConfig(), new RollerSim(getSimConstants()));
		}
	}
}
