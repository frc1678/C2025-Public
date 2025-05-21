package frc.robot.subsystems.superstructure;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.units.BaseUnits;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.lib.io.MotorIO.Setpoint;
import frc.robot.subsystems.algaedeploy.AlgaeDeploy;
import frc.robot.subsystems.algaedeploy.AlgaeDeployConstants;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.elevator.ElevatorConstants;
import frc.robot.subsystems.pivot.Pivot;
import frc.robot.subsystems.pivot.PivotConstants;
import java.util.Set;
import java.util.function.Supplier;

public class MotionPlanner {

	public static boolean pivotInHighClearRange(Angle pivotAngle) {
		return pivotAngle.gte(Units.Degrees.of(50.0)) && pivotAngle.lte(Units.Degrees.of(105.0));
	}

	public static boolean elevatorClearLowUp() {
		try {
			return Elevator.mInstance.nearPosition(
							ElevatorConstants.converter.toAngle(ElevatorConstants.kClearLowPosition))
					|| ElevatorConstants.converter
							.toDistance(Elevator.mInstance.getPosition())
							.gte(ElevatorConstants.kClearLowPosition);
		} catch (Exception e) {
			return false;
		}
	}

	public static boolean elevatorClearHighUp() {
		try {
			return Elevator.mInstance.nearPosition(
							ElevatorConstants.converter.toAngle(ElevatorConstants.kClearHighPosition))
					|| ElevatorConstants.converter
							.toDistance(Elevator.mInstance.getPosition())
							.gte(ElevatorConstants.kClearHighPosition);
		} catch (Exception e) {
			return false;
		}
	}

	public static boolean algaeDeployClearOut() {
		try {
			return AlgaeDeploy.mInstance.nearPosition(AlgaeDeployConstants.kClearPosition)
					|| AlgaeDeploy.mInstance.getPosition().lte(AlgaeDeployConstants.kClearPosition);
		} catch (Exception e) {
			return false;
		}
	}

	public static boolean pivotClearIn() {
		try {
			Angle pivotAngle = Pivot.mInstance.getPosition();
			return pivotAngle.gte(PivotConstants.kAlgaeImpactAngle) && pivotAngle.lte(PivotConstants.kCoralImpactAngle);
		} catch (Exception e) {
			return false;
		}
	}

	public static Command waitForElevatorClearLowUp() {
		return Commands.waitUntil(() -> elevatorClearLowUp());
	}

	public static Command waitForElevatorClearHighUp() {
		return Commands.waitUntil(() -> elevatorClearHighUp());
	}

	public static Command waitForAlgaeDeployClearOut() {
		return Commands.waitUntil(() -> algaeDeployClearOut());
	}

	public static Command waitForPivotClearIn() {
		Command c = Commands.waitUntil(() -> pivotClearIn());
		c.addRequirements(Pivot.mInstance);
		return c;
	}

	public static Command ensureElevatorClearUpLow() {
		Supplier<Command> supplier = () -> {
			Setpoint currentSetpoint = Elevator.mInstance.getSetpoint();
			if (currentSetpoint.mode.isPositionControl()
					&& currentSetpoint.baseUnits >= ElevatorConstants.kClearLowPosition.baseUnitMagnitude()) {
				return Commands.sequence(
						Elevator.mInstance.setpointCommand(currentSetpoint), waitForElevatorClearLowUp());
			} else {
				return Commands.sequence(
						Elevator.mInstance.setpointCommand(Elevator.CLEAR_LOW_HEIGHT), waitForElevatorClearLowUp());
			}
		};
		return Commands.defer(supplier, Set.of(Elevator.mInstance));
	}

	public static Command ensureElevatorClearUpHigh() {
		Supplier<Command> supplier = () -> {
			Setpoint currentSetpoint = Elevator.mInstance.getSetpoint();
			if (currentSetpoint.mode.isPositionControl()
					&& currentSetpoint.baseUnits >= ElevatorConstants.kClearHighPosition.baseUnitMagnitude()) {
				return Commands.sequence(
						Elevator.mInstance.setpointCommand(currentSetpoint), waitForElevatorClearHighUp());
			} else {
				return Commands.sequence(
						Elevator.mInstance.setpointCommand(Elevator.CLEAR_HIGH_HEIGHT), waitForElevatorClearHighUp());
			}
		};
		return Commands.defer(supplier, Set.of(Elevator.mInstance));
	}

	public static Command ensureAlgaeDeployClearOut() {
		Supplier<Command> supplier = () -> {
			Setpoint currentSetpoint = AlgaeDeploy.mInstance.getSetpoint();
			if (currentSetpoint.mode.isPositionControl()
					&& currentSetpoint.baseUnits <= AlgaeDeployConstants.kClearPosition.baseUnitMagnitude()) {
				return Commands.sequence(
						AlgaeDeploy.mInstance.setpointCommand(currentSetpoint), waitForAlgaeDeployClearOut());
			} else {
				return Commands.sequence(
						AlgaeDeploy.mInstance.setpointCommand(AlgaeDeploy.CLEAR), waitForAlgaeDeployClearOut());
			}
		};
		return Commands.defer(supplier, Set.of(AlgaeDeploy.mInstance));
	}

	public static Command stowPivot() {
		return Commands.sequence(ensurePivotFree(), Pivot.mInstance.setpointCommand(Pivot.CORAL_INTAKE));
	}

	public static Command ensurePivotFree() {
		return Commands.sequence(ensureAlgaeDeployClearOut(), ensureElevatorClearUpLow());
	}

	public static class UnsafePivotAndElevatorSynchronousToPositionMotionMagic extends Command {
		public final double baseFractionLookahead;

		private final double pivotSetpointBaseUnits;
		private final double elevatorSetpointBaseUnits;
		private double pivotTotalDeltaBaseUnits;
		private double elevatorTotalDeltaBaseUnits;
		private double pivotStartBaseUnits;
		private double elevatorStartBaseUnits;

		public UnsafePivotAndElevatorSynchronousToPositionMotionMagic(
				Angle pivotSetpoint, Distance elevatorSetpoint, double baseFractionLookahead) {
			addRequirements(Pivot.mInstance, Elevator.mInstance);

			pivotSetpointBaseUnits = pivotSetpoint.in(BaseUnits.AngleUnit);
			elevatorSetpointBaseUnits = elevatorSetpoint.in(BaseUnits.DistanceUnit);
			this.baseFractionLookahead = baseFractionLookahead;
		}

		@Override
		public void initialize() {
			pivotStartBaseUnits = Pivot.mInstance.getPosition().in(BaseUnits.AngleUnit);
			elevatorStartBaseUnits = ElevatorConstants.converter
					.toDistance(Elevator.mInstance.getPosition())
					.in(BaseUnits.DistanceUnit);

			pivotTotalDeltaBaseUnits = pivotSetpointBaseUnits - pivotStartBaseUnits;
			elevatorTotalDeltaBaseUnits = elevatorSetpointBaseUnits - elevatorStartBaseUnits;
		}

		@Override
		public void execute() {
			double currentPivotBaseUnits = Pivot.mInstance.getPosition().in(BaseUnits.AngleUnit);
			double currentElevatorBaseUnits = ElevatorConstants.converter
					.toDistance(Elevator.mInstance.getPosition())
					.in(BaseUnits.DistanceUnit);

			double pivotFractionIn = 1 - (pivotSetpointBaseUnits - currentPivotBaseUnits) / pivotTotalDeltaBaseUnits;
			double elevatorFractionIn =
					1 - (elevatorSetpointBaseUnits - currentElevatorBaseUnits) / elevatorTotalDeltaBaseUnits;

			Pivot.mInstance.applySetpoint(
					Setpoint.withMotionMagicSetpoint(BaseUnits.AngleUnit.of(pivotSetpointBaseUnits)));

			Elevator.mInstance.applySetpoint(Setpoint.withMotionMagicSetpoint(
					ElevatorConstants.converter.toAngle(BaseUnits.DistanceUnit.of(MathUtil.clamp(
							elevatorSetpointBaseUnits,
							elevatorStartBaseUnits
									- (Math.abs(elevatorTotalDeltaBaseUnits)
											* (pivotFractionIn + baseFractionLookahead)),
							elevatorStartBaseUnits
									+ (Math.abs(elevatorTotalDeltaBaseUnits)
											* (pivotFractionIn + baseFractionLookahead)))))));
		}

		@Override
		public boolean isFinished() {
			return Pivot.mInstance.nearPosition(BaseUnits.AngleUnit.of(pivotSetpointBaseUnits))
					&& Elevator.mInstance.nearPosition(
							ElevatorConstants.converter.toAngle(BaseUnits.DistanceUnit.of(elevatorSetpointBaseUnits)));
		}
	}

	public static Command safePivotAndElevatorToPosition(Setpoint pivotSetpoint, Setpoint elevatorSetpoint) {
		if (!pivotSetpoint.mode.isPositionControl()
				|| !elevatorSetpoint.mode.isPositionControl()) { // if either setpoint is not position control
			System.out.println("safePivotAndElevatorToPosition NEEDS TO USE POSITION CONTROL SETPOINTS");
			return null;
		}

		Angle pivotSetpointAngle = BaseUnits.AngleUnit.of(pivotSetpoint.baseUnits);
		Distance elevatorSetpointHeight =
				ElevatorConstants.converter.toDistance(BaseUnits.AngleUnit.of(elevatorSetpoint.baseUnits));

		Supplier<Command> supplier = () -> {
			if (elevatorClearHighUp()) { // If elevator is high clear
				if (elevatorSetpointHeight.gte(
						ElevatorConstants.kClearHighPosition)) { // If elevator is high clear and staying there
					return Commands.parallel(
							Pivot.mInstance.setpointCommandWithWait(pivotSetpoint),
							Elevator.mInstance.setpointCommandWithWait(elevatorSetpoint));
				} else { // if elevator is high clear but going down
					if (elevatorSetpointHeight.gte(ElevatorConstants.kClearLowPosition)) { // going above low stow
						return Commands.sequence(
								Commands.parallel(
										Pivot.mInstance.setpointCommand(pivotSetpoint),
										Elevator.mInstance.setpointCommand(Elevator.CLEAR_HIGH_HEIGHT)),
								ensureAlgaeDeployClearOut(),
								Commands.parallel(
										Elevator.mInstance.setpointCommandWithWait(elevatorSetpoint),
										Pivot.mInstance.waitForPositionCommand(pivotSetpointAngle)));
					} else { // going to low stow
						return Commands.sequence(
								Commands.parallel(
										Pivot.mInstance.setpointCommand(pivotSetpoint),
										Elevator.mInstance.setpointCommand(Elevator.CLEAR_HIGH_HEIGHT)),
								ensureAlgaeDeployClearOut(),
								Elevator.mInstance.setpointCommand(Elevator.CLEAR_LOW_HEIGHT),
								Pivot.mInstance.waitForPositionCommand(pivotSetpointAngle),
								Elevator.mInstance.setpointCommandWithWait(elevatorSetpoint));
					}
				}
			} else {
				if (elevatorSetpointHeight.gte(
						ElevatorConstants.kClearHighPosition)) { // if elevator is not high clear but going high clear

					if (pivotInHighClearRange(Pivot.mInstance.getPosition())) {
						return Commands.parallel(
								Elevator.mInstance.setpointCommandWithWait(elevatorSetpoint),
								Commands.sequence(
										waitForElevatorClearHighUp(),
										Pivot.mInstance.setpointCommandWithWait(pivotSetpoint)));
					} else {
						return Commands.sequence(
								ensureAlgaeDeployClearOut(),
								Commands.parallel(
										Elevator.mInstance.setpointCommandWithWait(elevatorSetpoint),
										Commands.sequence(
												waitForElevatorClearHighUp(),
												Pivot.mInstance.setpointCommandWithWait(pivotSetpoint))));
					}

				} else { // if elevator is down and staying down
					if ((Pivot.mInstance.getPosition().minus(pivotSetpointAngle).abs(Units.Degrees)
									< 10.0) // pivot doesnt need to move
							|| (pivotInHighClearRange(pivotSetpointAngle)
									&& pivotInHighClearRange(Pivot.mInstance
											.getPosition()))) { // Or it's gonna stay in the above angles that are clear
						return Commands.parallel(
								Pivot.mInstance.setpointCommandWithWait(pivotSetpoint),
								Elevator.mInstance.setpointCommandWithWait(elevatorSetpoint));
					} else {
						if (elevatorClearLowUp()) { // if elevator is above the lower clear position
							if (elevatorSetpointHeight.gte(
									ElevatorConstants.kClearLowPosition)) { // and staying above lower clear position
								return Commands.sequence(
										ensureAlgaeDeployClearOut(),
										Commands.parallel(
												Pivot.mInstance.setpointCommandWithWait(pivotSetpoint),
												Elevator.mInstance.setpointCommandWithWait(elevatorSetpoint)));
							} else { // and going below the lower clear position
								return Commands.sequence(
										ensureAlgaeDeployClearOut(),
										Elevator.mInstance.setpointCommand(Elevator.CLEAR_LOW_HEIGHT),
										Pivot.mInstance.setpointCommandWithWait(pivotSetpoint),
										Elevator.mInstance.setpointCommandWithWait(elevatorSetpoint));
							}
						} else { // if elevator is below the lower clear position
							if (elevatorSetpointHeight.gte(
									ElevatorConstants
											.kClearLowPosition)) { // and going to a point above lower clear postion
								return Commands.sequence(
										ensureAlgaeDeployClearOut(),
										Elevator.mInstance.setpointCommand(elevatorSetpoint),
										waitForElevatorClearLowUp(),
										Commands.parallel(
												Pivot.mInstance.setpointCommandWithWait(pivotSetpoint),
												Elevator.mInstance.waitForPositionCommand(
														ElevatorConstants.converter.toAngle(elevatorSetpointHeight))));
							} else { // and staying below lower clear position
								if (pivotInHighClearRange(pivotSetpointAngle)) { // if pivot going high clear
									return Commands.sequence(
											ensureAlgaeDeployClearOut(),
											Elevator.mInstance.setpointCommandWithWait(Elevator.CLEAR_LOW_HEIGHT),
											Pivot.mInstance.setpointCommand(pivotSetpoint),
											Commands.waitUntil(
													() -> pivotInHighClearRange(Pivot.mInstance.getPosition())),
											Elevator.mInstance.setpointCommandWithWait(elevatorSetpoint),
											Pivot.mInstance.waitForPositionCommand(pivotSetpointAngle));
								} else {
									return Commands.sequence(
											ensureAlgaeDeployClearOut(),
											Elevator.mInstance.setpointCommandWithWait(Elevator.CLEAR_LOW_HEIGHT),
											Pivot.mInstance.setpointCommandWithWait(pivotSetpoint),
											Elevator.mInstance.setpointCommandWithWait(elevatorSetpoint));
								}
							}
						}
					}
				}
			}
		};
		return Commands.sequence(
				Commands.defer(supplier, Set.of(Elevator.mInstance, Pivot.mInstance, AlgaeDeploy.mInstance)));
	}

	public static Command safeElevatorAndPivotToPositionWithoutClear(
			Setpoint pivotSetpoint, Setpoint elevatorSetpoint) {
		if (!pivotSetpoint.mode.isPositionControl()
				|| !elevatorSetpoint.mode.isPositionControl()) { // if either setpoint is not position control
			System.out.println("safePivotAndElevatorToPosition NEEDS TO USE POSITION CONTROL SETPOINTS");
			return null;
		}

		Distance elevatorSetpointHeight =
				ElevatorConstants.converter.toDistance(BaseUnits.AngleUnit.of(elevatorSetpoint.baseUnits));

		Supplier<Command> supplier = () -> {
			if (elevatorSetpointHeight.gte(ElevatorConstants.kClearHighPosition)) {
				return Commands.parallel(
						Elevator.mInstance.setpointCommandWithWait(elevatorSetpoint),
						Commands.sequence(
								waitForElevatorClearHighUp(), Pivot.mInstance.setpointCommandWithWait(pivotSetpoint)));
			} else {
				return Commands.sequence(
						Elevator.mInstance.setpointCommandWithWait(Elevator.CLEAR_HIGH_HEIGHT),
						Pivot.mInstance.setpointCommandWithWait(pivotSetpoint),
						Elevator.mInstance.setpointCommandWithWait(elevatorSetpoint));
			}
		};

		return Commands.sequence(Commands.defer(supplier, Set.of(Elevator.mInstance, Pivot.mInstance)));
	}
}
