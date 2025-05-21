package frc.robot.controlboard;

import com.ctre.phoenix6.swerve.SwerveRequest;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Command.InterruptionBehavior;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.lib.util.FieldLayout.Level;
import frc.robot.subsystems.algaedeploy.AlgaeDeploy;
import frc.robot.subsystems.algaedeploy.AlgaeDeployConstants;
import frc.robot.subsystems.climber.Climber;
import frc.robot.subsystems.climberrollers.ClimberRollers;
import frc.robot.subsystems.coralrollers.CoralRollers;
import frc.robot.subsystems.detection.Detection;
import frc.robot.subsystems.detection.DetectionConstants;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.DriveConstants;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.endeffector.EndEffector;
import frc.robot.subsystems.pivot.Pivot;
import frc.robot.subsystems.superstructure.Superstructure;
import java.util.function.Supplier;

public class ControlBoard extends SubsystemBase {
	public static final ControlBoard mInstance = new ControlBoard();

	private CommandXboxController driver = ControlBoardConstants.mDriverController;
	private CommandXboxController operator = ControlBoardConstants.mOperatorController;

	private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
	private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();

	private final Trigger overrideTrigger = driver.rightTrigger(0.1);
	private OverrideBehavior overrideBehavior = OverrideBehavior.CORAL_SCORE_L4;

	private Trigger rightBumper = driver.rightBumper();
	private Trigger endEffectorTrigger;

	public static enum OverrideBehavior {
		NET_SCORE(() -> Superstructure.mInstance.netScore().andThen(Superstructure.mInstance.tuck())),
		PROCESSOR_SCORE(
				() -> Superstructure.mInstance.processorScore().andThen(Superstructure.mInstance.tuckAfterProcessor())),
		ALGAE_HOLD(() -> Superstructure.mInstance.algaeStow()),
		CORAL_SCORE_L1(() -> Superstructure.mInstance.softCoralScore()),
		CORAL_SCORE_L2(() -> Superstructure.mInstance.coralScore(Level.L2)),
		CORAL_SCORE_L3(() -> Superstructure.mInstance.coralScore(Level.L3)),
		CORAL_SCORE_L4(() -> Superstructure.mInstance.coralScore(Level.L4)),
		TUCK(() -> Superstructure.mInstance.tuck()),
		NONE(() -> Commands.none());

		public final Supplier<Command> action;

		private OverrideBehavior(Supplier<Command> overrideAction) {
			action = overrideAction;
		}
	}

	public Command setOverrideBehavior(OverrideBehavior behavior) {
		return Commands.runOnce(() -> overrideBehavior = behavior);
	}

	public boolean getCoralMode() {
		return !rightBumper.getAsBoolean() && !Superstructure.mInstance.getHasAlgae();
	}

	public void configureBindings() {
		Drive.mInstance.setDefaultCommand(Drive.mInstance.followSwerveRequestCommand(
				DriveConstants.teleopRequest, DriveConstants.teleopRequestUpdater));
		driver.back()
				.onTrue(Commands.runOnce(
								() -> Drive.mInstance.getGeneratedDrive().seedFieldCentric(), Drive.mInstance)
						.ignoringDisable(true));

		driverControls();
		debugControls();
	}

	public OverrideBehavior getOverrideBehavior() {
		return overrideBehavior;
	}

	public void bringupControls() {
		operator.a().onTrue(Commands.runOnce(() -> Detection.mInstance.setPipeline(DetectionConstants.kAutoPipeline)));
		operator.b().onTrue(Commands.runOnce(() -> Detection.mInstance.setPipeline(DetectionConstants.kTelePipeline)));
	}

	public void driverControls() {
		Superstructure s = Superstructure.mInstance;

		// MISC ###############################################################################

		endEffectorTrigger = new Trigger(() -> s.getEndEffectorCoralBreak());
		endEffectorTrigger.onTrue(rumbleCommand(Units.Seconds.of(0.2)));

		driver.start().onTrue(s.stationIntakeToHold());

		driver.a().onTrue(s.spit().onlyWhile(driver.a()));

		driver.y().onTrue(s.prepClimb());

		driver.b().onTrue(s.stowClimb());

		driver.leftBumper().onTrue(s.tuckOrHold());

		overrideTrigger.onFalse(Commands.deferredProxy(() -> overrideBehavior.action.get()));

		// INTAKING ###############################################################################

		driver.leftTrigger(0.1)
				.onTrue(Commands.either(
								s.coralIntakeToHold()
										.asProxy()
										.beforeStarting(Commands.runOnce(() -> s.setForceGulp(false))),
								Commands.either(
										s.coralIntaketoIndexer(),
										s.algaeIntakeToHold()
												.asProxy()
												.beforeStarting(setOverrideBehavior(OverrideBehavior.ALGAE_HOLD)),
										() -> Superstructure.mInstance.getHasAlgae()),
								() -> getCoralMode())
						.withName("Either Coral Intake or Algae Intake"));

		// CORAL MODE ###############################################################################

		// Top Left Paddle
		bindCoralAutoScore(Level.L1, driver.povRight());

		// Top Right Paddle
		bindCoralAutoScore(Level.L2, driver.povUp());

		// Bottom Left Paddle
		bindCoralAutoScore(Level.L3, driver.povLeft());

		// Bottom Right Paddle
		bindCoralAutoScore(Level.L4, driver.povDown());

		// ALGAE MODE ###############################################################################

		// Top Left Paddle
		driver.povRight()
				.and(() -> !getCoralMode())
				.onTrue(Superstructure.mInstance.processorPrep())
				.onTrue(setOverrideBehavior(OverrideBehavior.PROCESSOR_SCORE));

		// Bottom Left Paddle
		bindAlgaeReefIntake(driver.povLeft());

		// Bottom Right Paddle
		bindNetAlignAndScore(driver.povDown());
	}

	public void bindCoralAutoScore(Level level, Trigger button) {
		Command scoreCommand =
				switch (level) {
					case L1 -> Superstructure.mInstance.L1Score();
					case L2 -> Superstructure.mInstance.L2Score();
					case L3 -> Superstructure.mInstance.L3Score();
					case L4 -> Superstructure.mInstance.L4Score();
					default -> Commands.none();
				};

		Command tuckCommand = level == Level.L1
				? Superstructure.mInstance.tuckAfterL1()
				: Superstructure.mInstance.tuckAfterScoring();

		OverrideBehavior coralOverrideBehavior =
				switch (level) {
					case L1 -> OverrideBehavior.CORAL_SCORE_L1;
					case L2 -> OverrideBehavior.CORAL_SCORE_L2;
					case L3 -> OverrideBehavior.CORAL_SCORE_L3;
					case L4 -> OverrideBehavior.CORAL_SCORE_L4;
					default -> OverrideBehavior.CORAL_SCORE_L4;
				};

		button.onTrue(Commands.either(
								Commands.sequence(
										Superstructure.mInstance
												.waitToStartScoreSequence()
												.onlyWhile(button)
												.until(overrideTrigger),
										scoreCommand
												.asProxy()
												.onlyWhile((button.or(overrideTrigger)))
												.onlyIf(button.or(overrideTrigger))
												.andThen(Commands.either(
														Commands.parallel(
																reefIntakeSuperstructure(driver.rightBumper()),
																reefIntakeDrive(driver.rightBumper()),
																reefIntakeSetOverrideBehavior(driver.rightBumper())),
														(tuckCommand)
																.asProxy()
																.onlyIf(button.and(
																		() -> EndEffector.mInstance.getCurrentCommand()
																				== null)),
														driver.rightBumper()))
												.asProxy()
												.withInterruptBehavior(InterruptionBehavior.kCancelIncoming)
												.withName("Auto Score " + level.toString())),
								Commands.none(),
								() -> getCoralMode())
						.withName("Either Auto Score " + level.toString()))
				.onTrue(Commands.either(
								Superstructure.mInstance
										.goToScoringPose(level)
										.asProxy()
										.until(overrideTrigger)
										.unless(overrideTrigger)
										.onlyWhile(button)
										.withName("Auto Align " + level.toString()),
								Commands.none(),
								() -> getCoralMode())
						.withName("Either Auto Align " + level.toString()))
				.onTrue(Commands.either(
						setOverrideBehavior(coralOverrideBehavior), Commands.none(), () -> getCoralMode()));
	}

	public void bindNetAlignAndScore(Trigger button) {
		button.onTrue(Commands.either(
								Commands.sequence(
										Superstructure.mInstance
												.waitUnitlSlowEnoughToRaiseNet()
												.onlyWhile(button)
												.until(overrideTrigger),
										Superstructure.mInstance
												.netPrep()
												.asProxy()
												.onlyWhile((button.or(overrideTrigger)))
												.onlyIf(button.or(overrideTrigger))
												.asProxy()
												.withInterruptBehavior(InterruptionBehavior.kCancelIncoming)
												.withName("Auto Score Net")),
								Commands.none(),
								() -> !getCoralMode())
						.withName("Either Auto Score Net Or None "))
				.onTrue(Commands.either(
								Superstructure.mInstance
										.alignToNet()
										.asProxy()
										.withInterruptBehavior(InterruptionBehavior.kCancelIncoming)
										.until(overrideTrigger)
										.unless(overrideTrigger)
										.onlyWhile(button)
										.withName("Auto Align Net Drive"),
								Commands.none(),
								() -> !getCoralMode())
						.withName("Either Auto Align Net Drive"))
				.onTrue(Commands.either(
						setOverrideBehavior(OverrideBehavior.NET_SCORE), Commands.none(), () -> !getCoralMode()));
	}

	public void bindAlgaeReefIntake(Trigger button) {
		button.onTrue(Commands.either(reefIntakeSuperstructure(button), Commands.none(), () -> !getCoralMode()))
				.onTrue(Commands.either(reefIntakeDrive(button), Commands.none(), () -> !getCoralMode()))
				.onTrue(Commands.either(reefIntakeSetOverrideBehavior(button), Commands.none(), () -> !getCoralMode()));
	}

	public Command reefIntakeSetOverrideBehavior(Trigger button) {
		return setOverrideBehavior(OverrideBehavior.ALGAE_HOLD)
				.onlyWhile(button)
				.until(overrideTrigger);
	}

	public Command reefIntakeSuperstructure(Trigger button) {
		return Commands.sequence(
						Superstructure.mInstance
								.reefIntakeAtRightLevel()
								.withInterruptBehavior(InterruptionBehavior.kCancelIncoming)
								.onlyIf(button.or(overrideTrigger).and(() -> !getCoralMode())),
						Superstructure.mInstance.stowAlgaeWhenReady())
				.asProxy()
				.withName("Either Algae Reef Intake SS");
	}

	public Command reefIntakeDrive(Trigger button) {
		return Superstructure.mInstance
				.goToReefIntakeReadyPose()
				.withDeadline(Superstructure.mInstance.elevatorAtRightPointForAlgaeIntakeFromReef())
				.andThen(Superstructure.mInstance.goToReefIntakePose())
				.asProxy()
				.until(overrideTrigger)
				.unless(overrideTrigger)
				.onlyWhile(button)
				.withName("Either Algae Reef Intake Drive");
	}

	public void bindProcessorAutoScore(Trigger button) {
		button.onTrue(Commands.either(
								new SequentialCommandGroup(
												Superstructure.mInstance
														.processorPrep()
														.onlyWhile(button.or(overrideTrigger)),
												Superstructure.mInstance
														.processorScoreWhenReady()
														.onlyWhile((button.or(overrideTrigger)))
														.onlyIf((button.or(overrideTrigger)))
														.andThen(Superstructure.mInstance
																.tuckAfterProcessor()
																.onlyIf(button.or(overrideTrigger))))
										.asProxy()
										.withName("Auto Processor Score"),
								Commands.none(),
								() -> !getCoralMode())
						.withName("Either Auto Processor Score"))
				.onTrue(Commands.either(
								Superstructure.mInstance
										.goToProcessorScoringPose()
										.until(overrideTrigger)
										.unless(overrideTrigger)
										.onlyWhile(button)
										.asProxy()
										.withName("Auto Align to Processor"),
								Commands.none(),
								() -> !getCoralMode())
						.withName("Either Auto Align to Processor"))
				.onTrue(Commands.either(
						setOverrideBehavior(OverrideBehavior.PROCESSOR_SCORE), Commands.none(), () -> !getCoralMode()));
	}

	private void debugControls() {
		Superstructure s = Superstructure.mInstance;

		operator.y().onTrue(s.stationIntakeToHold());
		operator.a()
				.onTrue(CoralRollers.mInstance.setpointCommand(CoralRollers.START))
				.onFalse(CoralRollers.mInstance.setpointCommand(CoralRollers.IDLE));

		operator.b()
				.onTrue(Climber.mInstance
						.setpointCommand(Climber.JOG_UP)
						.andThen(() -> Climber.mInstance.useSoftLimits(false)))
				.onFalse(Climber.mInstance
						.setpointCommand(Climber.HOLD)
						.andThen(() -> Climber.mInstance.useSoftLimits(true)));
		operator.x()
				.onTrue(Climber.mInstance
						.setpointCommand(Climber.JOG_DOWN)
						.andThen(() -> Climber.mInstance.useSoftLimits(false)))
				.onFalse(Climber.mInstance
						.setpointCommand(Climber.HOLD)
						.andThen(() -> Climber.mInstance.useSoftLimits(true)));

		operator.rightBumper()
				.onTrue(Elevator.mInstance
						.setpointCommand(Elevator.JOG_UP)
						.andThen(() -> Elevator.mInstance.useSoftLimits(false)))
				.onFalse(Elevator.mInstance
						.setpointCommand(Elevator.HOLD_UP)
						.andThen(() -> Elevator.mInstance.useSoftLimits(true)));
		operator.rightTrigger(0.1)
				.onTrue(Elevator.mInstance
						.setpointCommand(Elevator.JOG_DOWN)
						.andThen(() -> Elevator.mInstance.useSoftLimits(false)))
				.onFalse(Elevator.mInstance
						.setpointCommand(Elevator.HOLD_UP)
						.andThen(() -> Elevator.mInstance.useSoftLimits(true)));

		operator.povUp().onTrue(ClimberRollers.mInstance.setpointCommand(ClimberRollers.INTAKE));

		operator.povDown().onTrue(ClimberRollers.mInstance.setpointCommand(ClimberRollers.IDLE));

		operator.leftBumper()
				.onTrue(new InstantCommand(() -> Pivot.mInstance.setCurrentPosition(
								Pivot.mInstance.directCancoder.getPosition().getValue()))
						.ignoringDisable(true));

		operator.leftTrigger(0.1).onTrue(Elevator.mInstance.setpointCommand(Elevator.CLEAR_HIGH_HEIGHT));

		operator.back()
				.onTrue(Commands.sequence(
						Commands.either(
								Commands.none(),
								AlgaeDeploy.mInstance.setpointCommand(AlgaeDeploy.FAR_CLEAR),
								() -> AlgaeDeploy.mInstance
										.getPosition()
										.isNear(AlgaeDeployConstants.kFarClearPosition, 0.1)),
						Pivot.mInstance.setpointCommand(Pivot.JOG_POSITIVE)))
				.onFalse(Pivot.mInstance.setpointCommand(Pivot.HOLD));

		operator.start()
				.onTrue(Commands.sequence(
						Commands.either(
								Commands.none(),
								AlgaeDeploy.mInstance.setpointCommand(AlgaeDeploy.FAR_CLEAR),
								() -> AlgaeDeploy.mInstance
										.getPosition()
										.isNear(AlgaeDeployConstants.kFarClearPosition, 0.1)),
						Pivot.mInstance.setpointCommand(Pivot.JOG_NEGATIVE)))
				.onFalse(Pivot.mInstance.setpointCommand(Pivot.HOLD));
	}

	public Command rumbleCommand(Time duration) {
		return Commands.sequence(
						Commands.runOnce(() -> {
							setRumble(true);
						}),
						Commands.waitSeconds(duration.in(Units.Seconds)),
						Commands.runOnce(() -> {
							setRumble(false);
						}))
				.handleInterrupt(() -> {
					setRumble(false);
					;
				});
	}

	public void setRumble(boolean on) {
		ControlBoardConstants.mDriverController.getHID().setRumble(RumbleType.kBothRumble, on ? 1.0 : 0.0);
	}

	public void configureSysIDTests() {
		// Run SysId routines when holding back/start and X/Y.
		// Note that each routine should be run exactly once in a single log.
		driver.back()
				.and(driver.y())
				.whileTrue(Drive.mInstance.getGeneratedDrive().sysIdDynamic(Direction.kForward));
		driver.back()
				.and(driver.x())
				.whileTrue(Drive.mInstance.getGeneratedDrive().sysIdDynamic(Direction.kReverse));
		driver.start()
				.and(driver.y())
				.whileTrue(Drive.mInstance.getGeneratedDrive().sysIdQuasistatic(Direction.kForward));
		driver.start()
				.and(driver.x())
				.whileTrue(Drive.mInstance.getGeneratedDrive().sysIdQuasistatic(Direction.kReverse));

		// Reset the field-centric heading on left bumper press
		driver.leftBumper().onTrue(Drive.mInstance.getGeneratedDrive().runOnce(() -> Drive.mInstance
				.getGeneratedDrive()
				.seedFieldCentric()));
	}

	public void configureModulePointing() {
		driver.a().whileTrue(Drive.mInstance.getGeneratedDrive().applyRequest(() -> brake));
		driver.b()
				.whileTrue(Drive.mInstance
						.getGeneratedDrive()
						.applyRequest(() ->
								point.withModuleDirection(new Rotation2d(-driver.getLeftY(), -driver.getLeftX()))));
	}
}
