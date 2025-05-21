package frc.robot.subsystems.superstructure;

import com.ctre.phoenix6.swerve.SwerveDrivetrain.SwerveDriveState;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.units.BaseUnits;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.DeferredCommand;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.lib.drive.FollowSyncedPIDToPose;
import frc.lib.drive.FollowSyncedTagPIDToPose;
import frc.lib.drive.FollowXLineSwerveRequestCommand;
import frc.lib.drive.PIDToPoseCommand;
import frc.lib.io.BeamBreakIO;
import frc.lib.io.BeamBreakIOSim;
import frc.lib.io.MotorIO.Setpoint;
import frc.lib.util.FieldLayout;
import frc.lib.util.FieldLayout.Branch;
import frc.lib.util.FieldLayout.Branch.Face;
import frc.lib.util.FieldLayout.Level;
import frc.lib.util.Util;
import frc.robot.RobotConstants;
import frc.robot.controlboard.ControlBoard;
import frc.robot.subsystems.LEDs.LEDs;
import frc.robot.subsystems.algaedeploy.AlgaeDeploy;
import frc.robot.subsystems.algaedeploy.AlgaeDeployConstants;
import frc.robot.subsystems.algaerollers.AlgaeRollers;
import frc.robot.subsystems.climber.Climber;
import frc.robot.subsystems.climber.ClimberConstants;
import frc.robot.subsystems.climberrollers.ClimberRollers;
import frc.robot.subsystems.coraldeploy.CoralDeploy;
import frc.robot.subsystems.coraldeploy.CoralDeployConstants;
import frc.robot.subsystems.coralindexer.CoralIndexer;
import frc.robot.subsystems.coralrollers.CoralRollers;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.DriveConstants;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.elevator.ElevatorConstants;
import frc.robot.subsystems.endeffector.EndEffector;
import frc.robot.subsystems.pivot.Pivot;
import frc.robot.subsystems.pivot.PivotConstants;
import frc.robot.subsystems.superstructure.SuperstructureConstants.BeamBreakConstants;
import java.util.Set;
import java.util.function.BooleanSupplier;
import java.util.function.Supplier;

public class Superstructure extends SubsystemBase {
	public static final Superstructure mInstance = new Superstructure();

	public static BeamBreakIO endEffectorCoralBreak = BeamBreakConstants.getEndEffectorCoralBeamBreak();
	private static BeamBreakIO endEffectorAlgaeBreak = BeamBreakConstants.getEndEffectorAlgaeBeamBreak();
	public static BeamBreakIO endEffectorVelocityDip = BeamBreakConstants.getEndEffectorVelocityDip();
	public static BeamBreakIO coralRollersCurrentSpike = BeamBreakConstants.getCoralRollersCurrentSpike();
	public static BeamBreakIO coralRollersVelocityDip = BeamBreakConstants.getCoralRollersVelocityDip();
	public static BeamBreakIO climberRollersVelocityDip = BeamBreakConstants.getClimberRollersVelocityDip();
	public static BeamBreakIO indexerBreak = BeamBreakConstants.getIndexerBeamBreak();

	public static BeamBreakIO allAlgae = new BeamBreakIOSim(
			() -> endEffectorVelocityDip.getDebounced() || endEffectorAlgaeBreak.getDebounced(),
			Units.Seconds.of(0.0),
			"All Algae");

	private boolean algaeAligning = false;
	private boolean driveReady = false;
	private boolean superstructureDone = false;
	private boolean forceGulp = false;

	private boolean isPathFollowing = false;

	private boolean hasAlgae = false;

	private Branch targetingBranch = Branch.A;
	private Face targetingFace = targetingBranch.getKey().face();
	private boolean targetingL3ReefIntake = true;

	private State state = State.TUCK;

	public boolean readyToRaiseElevator = false;

	public void setForceGulp(boolean gulp) {
		forceGulp = gulp;
	}

	@Override
	public void periodic() {
		if (!isPathFollowing) {
			updateTargetedBranch();
			updateTargetedFace();
			updateTargetedReefIntake();
		}
	}

	public void updateTargetedBranch() {
		SwerveDriveState currentState = Drive.mInstance.getState();
		Transform2d speedsPose = new Transform2d(
						currentState.Speeds.vxMetersPerSecond,
						currentState.Speeds.vyMetersPerSecond,
						Rotation2d.fromRadians(currentState.Speeds.omegaRadiansPerSecond))
				.times(SuperstructureConstants.lookaheadBranchSelectionTime.in(Units.Seconds));
		Pose2d lookeaheadPose = currentState.Pose.transformBy(speedsPose);
		targetingBranch = FieldLayout.Branch.getClosestBranch(lookeaheadPose, RobotConstants.isRedAlliance);
	}

	public void updateTargetedFace() {
		targetingFace = targetingBranch.getKey().face();
	}

	public void updateTargetedReefIntake() {
		targetingL3ReefIntake = switch (targetingFace) {
			case NEAR_CENTER, FAR_LEFT, FAR_RIGHT -> true;
			case FAR_CENTER, NEAR_LEFT, NEAR_RIGHT -> false;};
	}

	@Override
	public void initSendable(SendableBuilder builder) {
		super.initSendable(builder);
		coralRollersCurrentSpike.initSendable(builder);
		endEffectorAlgaeBreak.initSendable(builder);
		endEffectorCoralBreak.initSendable(builder);
		endEffectorVelocityDip.initSendable(builder);
		coralRollersVelocityDip.initSendable(builder);
		climberRollersVelocityDip.initSendable(builder);
		allAlgae.initSendable(builder);
		indexerBreak.initSendable(builder);

		builder.addBooleanProperty("Is Coral Mode", () -> ControlBoard.mInstance.getCoralMode(), null);
		builder.addStringProperty("Targeting Branch", () -> targetingBranch.toString(), null);
		builder.addBooleanProperty("Motion Planner/Algae Clear Out", () -> MotionPlanner.algaeDeployClearOut(), null);
		builder.addBooleanProperty("Motion Planner/Elevator Clear Low", () -> MotionPlanner.elevatorClearLowUp(), null);
		builder.addBooleanProperty(
				"Motion Planner/Elevator Clear High", () -> MotionPlanner.elevatorClearHighUp(), null);
		builder.addBooleanProperty("Motion Planner/Pivot Clear In", () -> MotionPlanner.pivotClearIn(), null);
		builder.addBooleanProperty(
				"Motion Planner/Pivot Can Move",
				() -> (MotionPlanner.algaeDeployClearOut() && MotionPlanner.elevatorClearLowUp()),
				null);

		builder.addBooleanProperty("Drive Ready", () -> driveReady, null);
		builder.addBooleanProperty("Superstructure Done", () -> superstructureDone, null);
		builder.addBooleanProperty("Algae Aligning", () -> algaeAligning, null);
		builder.addStringProperty(
				"Override Behavior",
				() -> ControlBoard.mInstance.getOverrideBehavior().toString(),
				null);
		builder.addStringProperty("State", () -> state.toString(), null);
		builder.addBooleanProperty("Force Gulp", () -> forceGulp, null);
		builder.addBooleanProperty("Has Algae", () -> hasAlgae, null);
		builder.addBooleanProperty("Ready To Raise Elevator", () -> readyToRaiseElevator, null);
		builder.addBooleanProperty("Clear From Processor", () -> getClearFromProcessor(), null);

		builder.addDoubleProperty("Battery Voltage", () -> RobotController.getBatteryVoltage(), null);
	}

	public Command tuckOrHold() {
		return Commands.either(stowCoralHold(), tuck(), () -> endEffectorCoralBreak.getDebounced());
	}

	/*
	 * Stops coral rollers, coral indexer, and algae rollers
	 */
	public Command idleIntakes() {
		return Commands.parallel(
						CoralRollers.mInstance.setpointCommand(CoralRollers.IDLE),
						AlgaeRollers.mInstance.setpointCommand(AlgaeRollers.IDLE),
						CoralIndexer.mInstance.setpointCommand(CoralIndexer.IDLE))
				.withName("Idle Intakes");
	}

	public Command stowIntakes() {
		return Commands.parallel(
						AlgaeDeploy.mInstance.setpointCommandWithWait(AlgaeDeploy.STOW),
						CoralDeploy.mInstance.setpointCommandWithWait(CoralDeploy.STOW_CLEAR))
				.withName("Stow Intakes");
	}

	/*
	 * Subsystem zero(ing) method
	 */
	public Command zero() {
		return Commands.runOnce(() -> {
					AlgaeDeploy.mInstance.setCurrentPosition(AlgaeDeployConstants.kStowPosition);
					CoralDeploy.mInstance.setCurrentPosition(CoralDeployConstants.kStowClearPosition);
					Climber.mInstance.setCurrentPosition(
							ClimberConstants.converter.toAngle(ClimberConstants.kStowPosition));
					Elevator.mInstance.setCurrentPosition(
							ElevatorConstants.converter.toAngle(ElevatorConstants.kStowPosition));
				})
				.withName("Zero");
	}

	/**
	 * Spins the coral rollers for a bit, used to be used at the beginning of our autos when
	 * we tucked the intake in via a string to fit within frame perimeter
	 */
	public Command liberateCoralDeploy() {
		return Commands.sequence(
						CoralRollers.mInstance.setpointCommand(CoralRollers.START),
						Commands.waitSeconds(0.5),
						CoralRollers.mInstance.setpointCommand(CoralRollers.IDLE))
				.withName("Liberate Coral Deploy");
	}

	public Command spit() {
		return Commands.parallel(
						CoralDeploy.mInstance.setpointCommand(CoralDeploy.EXHAUST),
						CoralRollers.mInstance.setpointCommand(CoralRollers.EXHAUST),
						CoralIndexer.mInstance.setpointCommand(CoralIndexer.EXHAUST),
						EndEffector.mInstance.setpointCommand(EndEffector.SPIT),
						setState(State.SPIT),
						setHasAlgaeCommand(false),
						Commands.waitUntil(() -> false))
				.handleInterrupt(() -> {
					CoralRollers.mInstance.applySetpoint(CoralRollers.IDLE);
					CoralIndexer.mInstance.applySetpoint(CoralIndexer.IDLE);
					EndEffector.mInstance.applySetpoint(EndEffector.IDLE);
				})
				.withName("Spit");
	}

	/**
	 * Brings a coral to our "hold" position, which allows us to get to coral scoring sequences faster
	 */
	public Command stowCoralHold() {
		return Commands.sequence(
						Commands.parallel(
								EndEffector.mInstance.setpointCommand(EndEffector.CORAL_HOLD),
								MotionPlanner.safePivotAndElevatorToPosition(Pivot.CORAL_HOLD, Elevator.CORAL_HOLD)),
						setState(State.HOLD_CORAL),
						AlgaeDeploy.mInstance.setpointCommand(AlgaeDeploy.STOW))
				.withName("Stow Coral Hold");
	}

	/**
	 * Specific sequence for L1 scoring; makes sure the algae deploy stays out so that the L1 scoring doesn't fall early
	 */
	public Command waitToPullAlgaeDeployBackInAfterL1() {
		return Commands.waitUntil(
				() -> Util.getDistanceFromReef().gte(SuperstructureConstants.farFromReefAfterL1Threshold)
						|| Math.abs(Util.getAngleFromReef().getDegrees())
								> SuperstructureConstants.lookingAwayFromReefAfterL1Threshold.in(Units.Degrees));
	}

	public Command waitUntilSafeToProcessorUnjam() {
		return Commands.waitUntil(() -> getClearFromProcessor());
	}

	public Command waitToStopEjectingEndEffector() {
		return Commands.waitUntil(
				() -> Pivot.mInstance.getPosition().lte(PivotConstants.kEndEffectorIdleAfterScoringAngle));
	}

	public Command waitUnitlSlowEnoughToRaiseNet() {
		return Commands.waitUntil(() -> Math.hypot(
								Drive.mInstance.getState().Speeds.vxMetersPerSecond,
								Drive.mInstance.getState().Speeds.vyMetersPerSecond)
						< DriveConstants.kMaxSpeedVeryTippy.times(1.1).in(Units.MetersPerSecond))
				.alongWith(Commands.waitUntil(() -> closeToNetLine()));
	}

	public Command waitUnitlSlowEnoughToRaiseNetInAuto() {
		return Commands.waitUntil(() -> Math.hypot(
								Drive.mInstance.getState().Speeds.vxMetersPerSecond,
								Drive.mInstance.getState().Speeds.vyMetersPerSecond)
						< DriveConstants.kMaxSpeedVeryTippy.times(1.1).in(Units.MetersPerSecond))
				.alongWith(Commands.waitUntil(() -> closeToNetLineInAuto()));
	}

	/**
	 * Stows every subsystem
	 */
	public Command tuck() {
		return Commands.parallel(
						idleIntakes(),
						EndEffector.mInstance.setpointCommand(EndEffector.IDLE),
						Commands.sequence(
								MotionPlanner.safePivotAndElevatorToPosition(Pivot.CORAL_INTAKE, Elevator.STOW),
								Commands.parallel(
										AlgaeDeploy.mInstance.setpointCommandWithWait(AlgaeDeploy.STOW),
										CoralDeploy.mInstance
												.setpointCommandWithWait(CoralDeploy.STOW_CLEAR)
												.onlyIf(() -> !indexerBreak.get()),
										setState(State.TUCK))))
				.withName("Tuck");
	}

	public Command tuckAfterScoring() {
		return Commands.parallel(
						idleIntakes(),
						Commands.sequence(
								MotionPlanner.safePivotAndElevatorToPosition(Pivot.CORAL_INTAKE, Elevator.STOW),
								Commands.parallel(
										AlgaeDeploy.mInstance.setpointCommandWithWait(AlgaeDeploy.STOW),
										CoralDeploy.mInstance.setpointCommandWithWait(CoralDeploy.STOW_CLEAR),
										setState(State.TUCK))),
						Commands.sequence(
								waitToStopEjectingEndEffector(),
								EndEffector.mInstance.setpointCommand(EndEffector.IDLE)))
				.withName("Tuck After Scoring");
	}

	/**
	 * Specific sequence for L1 scoring; different tuck sequence since we're in a different state when compared to other scores
	 */
	public Command tuckAfterL1() {
		return Commands.parallel(
						EndEffector.mInstance.setpointCommand(EndEffector.IDLE),
						Commands.sequence(
								MotionPlanner.safePivotAndElevatorToPosition(Pivot.CORAL_INTAKE, Elevator.STOW),
								Commands.parallel(
										Commands.sequence(
												waitToPullAlgaeDeployBackInAfterL1(),
												AlgaeDeploy.mInstance.setpointCommandWithWait(AlgaeDeploy.STOW),
												AlgaeRollers.mInstance.setpointCommand(AlgaeRollers.IDLE)),
										CoralDeploy.mInstance.setpointCommandWithWait(CoralDeploy.STOW_CLEAR),
										setState(State.TUCK))))
				.withName("Tuck")
				.handleInterrupt(() -> {
					AlgaeRollers.mInstance.applySetpoint(AlgaeRollers.IDLE);
					AlgaeDeploy.mInstance.applySetpoint(AlgaeDeploy.CLEAR);
				});
	}

	/**
	 * Specific sequence for processor scoring; different tuck sequence since we're in a different state when compared to other scores
	 */
	public Command tuckAfterProcessor() {
		return Commands.parallel(
						idleIntakes(),
						EndEffector.mInstance.setpointCommand(EndEffector.IDLE),
						Commands.sequence(
								waitUntilSafeToProcessorUnjam(),
								AlgaeDeploy.mInstance.setpointCommand(AlgaeDeploy.EE_UNJAM),
								MotionPlanner.safePivotAndElevatorToPosition(Pivot.CORAL_INTAKE, Elevator.EE_UNJAM),
								AlgaeDeploy.mInstance.setpointCommandWithWait(AlgaeDeploy.EE_UNJAM),
								MotionPlanner.safePivotAndElevatorToPosition(Pivot.CORAL_INTAKE, Elevator.STOW),
								Commands.parallel(
										AlgaeDeploy.mInstance.setpointCommandWithWait(AlgaeDeploy.STOW),
										CoralDeploy.mInstance.setpointCommandWithWait(CoralDeploy.STOW_CLEAR),
										setState(State.TUCK))))
				.withName("Tuck After Processor");
	}

	/**
	 * Intakes an algae and holds it
	 */
	public Command algaeIntakeToHold() {
		return Commands.sequence(
						AlgaeDeploy.mInstance.setpointCommand(AlgaeDeploy.DEPLOY),
						MotionPlanner.safePivotAndElevatorToPosition(Pivot.ALGAE_INTAKE, Elevator.ALGAE_FEED_HEIGHT),
						AlgaeRollers.mInstance
								.getSpinForIntakeWithUnstuckCommand()
								.withDeadline(Commands.sequence(
										EndEffector.mInstance.setpointCommand(EndEffector.ALGAE_FEED),
										setState(State.GROUND_ALGAE),
										Commands.waitSeconds(0.2),
										allAlgae.stateWait(true),
										setHasAlgaeCommand(true),
										EndEffector.mInstance.setpointCommand(EndEffector.ALGAE_HOLD))),
						AlgaeRollers.mInstance.setpointCommand(AlgaeRollers.IDLE),
						MotionPlanner.safePivotAndElevatorToPosition(Pivot.ALGAE_HOLD, Elevator.ALGAE_HOLD),
						setState(State.HOLD_ALGAE),
						AlgaeDeploy.mInstance.setpointCommand(AlgaeDeploy.STOW))
				.withName("Algae Intake To Hold")
				.handleInterrupt(() -> {
					AlgaeRollers.mInstance.applySetpoint(AlgaeRollers.IDLE);
				});
	}

	/**
	 * Includes the whole intake sequence, and ensures that it exhausts to avoid intaking a
	 * second coral along with raising the elevator to the hold position automatically
	 */
	public Command coralIntakeToHold() {
		return Commands.sequence(
						coralIntakeToEndEffector(),
						Commands.parallel(
								LEDs.mInstance.flashCommand(),
								new Util.ScheduleIfWontCancelOther(
										stowCoralHold().asProxy()),
								new Util.ScheduleIfWontCancelOther(
										exhaustCoralIntake().asProxy())),
						setState(State.HOLD_CORAL))
				.withName("Coral Intake To Hold");
	}

	/**
	 * See coralIntakeToHold(); only the part that intakes, stops once the end effector break is asserted.
	 * Used in autos for switching from ground intake part to scoring part.
	 */
	public Command coralIntakeToEndEffector() {
		return Commands.sequence(
						Commands.parallel(
								setState(State.GROUND_CORAL),
								AlgaeDeploy.mInstance.setpointCommand(AlgaeDeploy.CLEAR),
								EndEffector.mInstance.setpointCommand(EndEffector.CORAL_FEED),
								CoralDeploy.mInstance.setpointCommand(CoralDeploy.DEPLOY),
								CoralRollers.mInstance.setpointCommand(CoralRollers.INTAKE)),
						Commands.sequence(
								MotionPlanner.safePivotAndElevatorToPosition(Pivot.CORAL_INTAKE, Elevator.STOW),
								CoralIndexer.mInstance.setpointCommand(CoralIndexer.INTAKE)))
				.withDeadline(endEffectorCoralBreak.stateWaitWithDebounceIfReal(true, 1.5))
				.finallyDo(() -> {
					CoralRollers.mInstance.applySetpoint(CoralRollers.IDLE);
					CoralIndexer.mInstance.applySetpoint(CoralIndexer.IDLE);
					EndEffector.mInstance.applySetpoint(EndEffector.CORAL_HOLD);
					CoralDeploy.mInstance.applySetpoint(CoralDeploy.EXHAUST);
				});
	}

	/**
	 * Keeps the coral held in the indexer instead of taking it all the way to the end effector so that
	 * we can hold two gamepieces
	 */
	public Command coralIntaketoIndexer() {
		return Commands.sequence(
						Commands.parallel(
								setState(State.GROUND_CORAL_WITH_ALGAE),
								CoralDeploy.mInstance.setpointCommand(CoralDeploy.DEPLOY),
								CoralRollers.mInstance.setpointCommand(CoralRollers.INTAKE)),
						indexerBreak.stateWait(true),
						Commands.parallel(
								CoralDeploy.mInstance.setpointCommand(CoralDeploy.INDEXERHOLD),
								CoralRollers.mInstance.setpointCommand(CoralRollers.IDLE)))
				.withName("Coral Intake with Algae");
	}

	public Command exhaustCoralIntake() {
		return Commands.sequence(
						Commands.parallel(
								CoralDeploy.mInstance.setpointCommand(CoralDeploy.EXHAUST),
								CoralIndexer.mInstance.setpointCommand(CoralIndexer.EXHAUST),
								CoralRollers.mInstance.setpointCommand(CoralRollers.EXHAUST)),
						Commands.waitTime(Units.Seconds.of(1.0)),
						CoralRollers.mInstance.setpointCommand(CoralRollers.IDLE),
						CoralIndexer.mInstance.setpointCommand(CoralIndexer.IDLE),
						CoralDeploy.mInstance.setpointCommand(CoralDeploy.STOW_CLEAR))
				.handleInterrupt(() -> {
					CoralRollers.mInstance.applySetpoint(CoralRollers.IDLE);
					CoralIndexer.mInstance.applySetpoint(CoralIndexer.IDLE);
					CoralDeploy.mInstance.applySetpoint(CoralDeploy.STOW_CLEAR);
				});
	}

	/**
	 * Spins end effector in coral scoring direction
	 */
	public Command coralScore(Level level) {
		return Commands.sequence(
						EndEffector.mInstance.setpointCommand(EndEffector.getCoralScoreSetpoint(level)),
						endEffectorCoralBreak.stateWaitWithDebounceIfReal(false, 0.0),
						Commands.waitTime(
								level == Level.L3 || level == Level.L2
										? Units.Seconds.of(0.18)
										: Units.Seconds.of(0.05)),
						Commands.parallel(
								Commands.runOnce(() -> setSuperstructureDone(true)), LEDs.mInstance.flashCommand()))
				.handleInterrupt(() -> {
					Util.smartDashCommand("Superstructure/Coral Score Last Interrupted");
				})
				.withName("Coral Score");
	}

	public Command softCoralScore() {
		return Commands.sequence(
						EndEffector.mInstance.setpointCommand(EndEffector.SOFT_CORAL_SCORE),
						AlgaeRollers.mInstance.setpointCommand(AlgaeRollers.L1_SOFT_SPIT),
						endEffectorCoralBreak.stateWaitWithDebounceIfReal(false, 0.0),
						Commands.waitTime(Units.Seconds.of(0.4)),
						Commands.parallel(
								Commands.runOnce(() -> setSuperstructureDone(true)),
								EndEffector.mInstance.setpointCommand(EndEffector.IDLE)),
						LEDs.mInstance.flashCommand())
				.handleInterrupt(() -> {
					EndEffector.mInstance.applySetpoint(EndEffector.IDLE);
					AlgaeRollers.mInstance.applySetpoint(AlgaeRollers.IDLE);
				})
				.withName("Soft Coral Score");
	}

	public Command L1Score() {
		return Commands.sequence(L1Prep(), scoreCoralWhenReady(Level.L1)).withName("L1 Score When Ready");
	}

	public Command L2Score() {
		return Commands.sequence(L2Prep(), scoreCoralWhenReady(Level.L2)).withName("L2 Score When Ready");
	}

	public Command L3Score() {
		return Commands.sequence(L3Prep(), scoreCoralWhenReady(Level.L3)).withName("L3 Score When Ready");
	}

	public Command L4Score() {
		return Commands.sequence(L4Prep(), scoreCoralWhenReady(Level.L4)).withName("L4 Score When Ready");
	}

	public Command L4ScoreInAuto() {
		return Commands.sequence(L4Prep(), waitUntilDriveReadyToScoreAndStable())
				.withTimeout(Units.Seconds.of(2.5))
				.andThen(coralScore(Level.L4))
				.withName("L4 Score When Ready");
	}

	public Command L3ScoreInAuto() {
		return Commands.sequence(L3Prep(), waitUntilDriveReadyToScoreAndStable())
				.withTimeout(Units.Seconds.of(2.5))
				.andThen(coralScore(Level.L3))
				.withName("L3 Score When Ready");
	}

	public Command L2ScoreInAuto() {
		return Commands.sequence(L2Prep(), waitUntilDriveReadyToScoreAndStable())
				.withTimeout(Units.Seconds.of(4.0))
				.andThen(coralScore(Level.L2))
				.withName("L2 Score When Ready");
	}

	/**
	 * Prepare elevator and pivot for L1 score
	 */
	public Command L1Prep() {
		return Commands.sequence(
						EndEffector.mInstance.setpointCommand(EndEffector.CORAL_HOLD),
						MotionPlanner.safePivotAndElevatorToPosition(Pivot.L1_SCORE, Elevator.L1_SCORE),
						setState(State.L1_CORAL),
						AlgaeDeploy.mInstance.setpointCommand(AlgaeDeploy.L1_SCORE))
				.withName("L1 Prep");
	}

	/**
	 * Prepare elevator and pivot for L2 score
	 */
	public Command L2Prep() {
		return Commands.sequence(
						EndEffector.mInstance.setpointCommand(EndEffector.CORAL_HOLD),
						MotionPlanner.safePivotAndElevatorToPosition(Pivot.CORAL_HOLD, Elevator.L2_SCORE)
								.unless(() -> elevatorAtOrBelowL2())
								.until(() -> elevatorAtOrBelowL2()),
						MotionPlanner.safePivotAndElevatorToPosition(Pivot.L2_SCORE, Elevator.L2_SCORE),
						setState(State.L2_CORAL),
						AlgaeDeploy.mInstance.setpointCommand(AlgaeDeploy.STOW))
				.withName("L2 Prep");
	}

	/**
	 * Prepare elevator and pivot for L3 score
	 */
	public Command L3Prep() {
		return Commands.sequence(
						EndEffector.mInstance.setpointCommand(EndEffector.CORAL_HOLD),
						MotionPlanner.safePivotAndElevatorToPosition(Pivot.L3_SCORE, Elevator.L3_SCORE),
						setState(State.L3_CORAL),
						AlgaeDeploy.mInstance.setpointCommand(AlgaeDeploy.STOW))
				.withName("L3 Prep");
	}

	/**
	 * Prepare elevator and pivot for L4 score
	 */
	public Command L4Prep() {
		return Commands.sequence(
						Commands.parallel(
								EndEffector.mInstance.setpointCommand(EndEffector.CORAL_HOLD),
								MotionPlanner.safePivotAndElevatorToPosition(Pivot.CORAL_HOLD, Elevator.L4_CLEAR)
										.onlyIf(() -> !getPivotNearOrAboveHoldPosition())
										.onlyWhile(() -> !getPivotNearOrAboveHoldPosition())),
						MotionPlanner.safePivotAndElevatorToPosition(Pivot.CORAL_HOLD, Elevator.L4_SCORE)
								.until(() -> Elevator.mInstance
										.getPosition()
										.gte(ElevatorConstants.converter.toAngle(
												ElevatorConstants.kL4PivotClearHeight)))
								.unless(() -> Elevator.mInstance
										.getPosition()
										.gte(ElevatorConstants.converter.toAngle(
												ElevatorConstants.kL4PivotClearHeight))),
						MotionPlanner.safePivotAndElevatorToPosition(Pivot.L4_SCORE, Elevator.L4_SCORE),
						AlgaeDeploy.mInstance.setpointCommand(AlgaeDeploy.STOW),
						setState(State.L4_CORAL))
				.withName("L4 Prep");
	}

	public boolean getPivotNearOrAboveHoldPosition() {
		return Pivot.mInstance.nearPosition(PivotConstants.kCoralHold)
				|| Pivot.mInstance.getPosition().gte(PivotConstants.kCoralHold);
	}

	/**
	 * Prepare elevator and pivot for net score
	 */
	public Command netPrep() {
		return Commands.sequence(
						MotionPlanner.safePivotAndElevatorToPosition(Pivot.NET_SCORE, Elevator.NET_PREP)
								.unless(() -> Pivot.mInstance.nearPosition(PivotConstants.kNetScore))
								.until(() -> Pivot.mInstance.nearPosition(PivotConstants.kNetScore)),
						MotionPlanner.safePivotAndElevatorToPosition(Pivot.NET_SCORE, Elevator.NET_HEIGHT),
						setState(State.NET),
						AlgaeDeploy.mInstance.setpointCommandWithWait(AlgaeDeploy.STOW),
						EndEffector.mInstance.setpointCommand(EndEffector.ALGAE_HOLD))
				.withName("Net Prep");
	}

	public Command netScore() {
		return Commands.sequence(
						EndEffector.mInstance.setpointCommand(EndEffector.NET_ALGAE_SCORE),
						setHasAlgaeCommand(false),
						Commands.waitSeconds(0.3),
						EndEffector.mInstance.setpointCommand(EndEffector.IDLE))
				.handleInterrupt(() -> setHasAlgae(false));
	}

	public Command netScoreInAuto() {
		return Commands.sequence(
						EndEffector.mInstance.setpointCommand(Setpoint.withVoltageSetpoint(Units.Volts.of(-9.0))),
						setHasAlgaeCommand(false),
						Commands.waitSeconds(0.3),
						EndEffector.mInstance.setpointCommand(EndEffector.IDLE))
				.handleInterrupt(() -> setHasAlgae(false));
	}

	public Command lolipopIntake() {
		return Commands.sequence(
				MotionPlanner.safePivotAndElevatorToPosition(Pivot.ALGAE_INTAKE, Elevator.LOLIPOP),
				setState(State.LOLLIPOP_ALGAE),
				EndEffector.mInstance.setpointCommand(EndEffector.ALGAE_INTAKE),
				endEffectorAlgaeBreak.stateWaitWithDebounce(false),
				EndEffector.mInstance.setpointCommand(EndEffector.ALGAE_HOLD),
				algaeStow());
	}

	/**
	 * Prepare elevator and pivot for L2 algae intake, then grab algae
	 */
	public Command reefAlgaeIntake(boolean isL3) {
		return Commands.sequence(
				Commands.parallel(
						Commands.sequence(
								waitToStopEjectingEndEffector(),
								EndEffector.mInstance.setpointCommand(EndEffector.ALGAE_INTAKE)),
						Commands.sequence(
								MotionPlanner.safePivotAndElevatorToPosition(
										Pivot.REEF_PREP, isL3 ? Elevator.L3_ALGAE : Elevator.L2_ALGAE),
								Pivot.mInstance.setpointCommand(Pivot.REEF_INTAKE))),
				setState(isL3 ? State.L3_ALGAE : State.L2_ALGAE),
				AlgaeDeploy.mInstance.setpointCommand(AlgaeDeploy.STOW),
				allAlgae.stateWait(true),
				Commands.parallel(
						setHasAlgaeCommand(true),
						EndEffector.mInstance
								.setpointCommand(EndEffector.ALGAE_HOLD)
								.beforeStarting(Commands.waitSeconds(0.2)),
						Elevator.mInstance.setpointCommandWithWait(isL3 ? Elevator.L3_LIFT : Elevator.L2_LIFT)));
	}

	/**
	 * Prepare elevator and pivot for L2 algae intake, then grab algae
	 */
	public Command reefAlgaeIntakeInAuto(boolean isL3) {
		return Commands.sequence(
				EndEffector.mInstance.setpointCommand(EndEffector.ALGAE_INTAKE),
				MotionPlanner.safePivotAndElevatorToPosition(
						Pivot.REEF_PREP, isL3 ? Elevator.L3_ALGAE : Elevator.L2_ALGAE),
				Pivot.mInstance.setpointCommand(Pivot.REEF_INTAKE),
				setState(isL3 ? State.L3_ALGAE : State.L2_ALGAE),
				AlgaeDeploy.mInstance.setpointCommand(AlgaeDeploy.STOW),
				endEffectorAlgaeBreak.stateWaitWithDebounceIfReal(true, 1.5));
	}

	public Command algaeStow() {
		return Commands.sequence(
				EndEffector.mInstance.setpointCommand(EndEffector.ALGAE_HOLD),
				MotionPlanner.safePivotAndElevatorToPosition(Pivot.REEF_INTAKE, Elevator.ALGAE_HOLD),
				MotionPlanner.safePivotAndElevatorToPosition(Pivot.ALGAE_HOLD, Elevator.ALGAE_HOLD),
				setState(State.HOLD_ALGAE));
	}

	/**
	 * Prepare to score algae in processor
	 */
	public Command processorPrep() {
		return Commands.sequence(
				AlgaeDeploy.mInstance.setpointCommandWithWait(AlgaeDeploy.ALGAE_CLEAR),
				MotionPlanner.safePivotAndElevatorToPosition(Pivot.PROCESSOR, Elevator.PROCESSOR),
				AlgaeDeploy.mInstance.setpointCommand(AlgaeDeploy.PROCESSOR),
				setState(State.PROCESSOR));
	}

	/**
	 * Prepare then score algae in processor
	 */
	public Command processorScore() {
		return Commands.sequence(
						Commands.parallel(
								EndEffector.mInstance.setpointCommand(EndEffector.PROCESSOR_ALGAE_SCORE),
								AlgaeRollers.mInstance.setpointCommand(AlgaeRollers.EXHAUST)),
						setHasAlgaeCommand(false),
						Commands.waitSeconds(0.5),
						Commands.parallel(
								EndEffector.mInstance.setpointCommand(EndEffector.IDLE),
								AlgaeRollers.mInstance.setpointCommand(AlgaeRollers.IDLE),
								Util.smartDashCommand("PROC FINISH"),
								Commands.runOnce(() -> setSuperstructureDone(true))))
				.handleInterrupt(() -> {
					EndEffector.mInstance.setpointCommand(EndEffector.IDLE);
					AlgaeRollers.mInstance.setpointCommand(AlgaeRollers.IDLE);
					setHasAlgae(false);
				});
	}

	public Command processorScoreWhenReady() {
		return Commands.waitUntil(() -> getDriveReady()).andThen(processorScore());
	}

	/**
	 * Send the climber hooks into the cage and hoist up robot
	 */
	public Command prepClimb() {
		return Commands.sequence(
				tuck(),
				Commands.parallel(
						Climber.mInstance.setpointCommand(Climber.PREP),
						ClimberRollers.mInstance.setpointCommand(ClimberRollers.INTAKE),
						CoralDeploy.mInstance.setpointCommand(CoralDeploy.STOW_CLEAR)),
				Commands.waitSeconds(1.5),
				climberRollersVelocityDip.stateWaitWithDebounce(true),
				Commands.waitSeconds(0.5),
				Commands.parallel(
						ControlBoard.mInstance.rumbleCommand(Units.Seconds.of(0.5)), LEDs.mInstance.flashCommand()));
	}

	/**
	 * Hoist up robot
	 */
	public Command stowClimb() {
		return Commands.parallel(
						Climber.mInstance.setpointCommandWithWait(Climber.PULL),
						ClimberRollers.mInstance.setpointCommand(ClimberRollers.IDLE))
				.andThen(CoralDeploy.mInstance.setpointCommandWithWait(CoralDeploy.STOW_FULL));
	}

	public Command goToScoringPose(Level level) {
		return Commands.sequence(Commands.defer(
				() -> {
					if (level == Level.L1) {
						Branch target = targetingBranch;
						Pose2d finalL1Pose = FieldLayout.handleAllianceFlip(
								FieldLayout.getCoralScoringPose(target)
										.transformBy(new Transform2d(
												SuperstructureConstants.kL1CoralOffsetFactor.unaryMinus(),
												target.getKey().isLeft()
														? SuperstructureConstants.kL1CoralHorizontalOffsetFactor
																.unaryMinus()
														: SuperstructureConstants.kL1CoralHorizontalOffsetFactor,
												new Rotation2d())),
								RobotConstants.isRedAlliance);

						return new FollowSyncedTagPIDToPose(finalL1Pose, level, true);
					} else {
						Pose2d scoringPose = FieldLayout.handleAllianceFlip(
								FieldLayout.getCoralScoringPose(targetingBranch), RobotConstants.isRedAlliance);
						return new FollowSyncedTagPIDToPose(scoringPose, level);
					}
				},
				Set.of(Drive.mInstance)));
	}

	public Command goToProcessorScoringPose() {
		return Commands.sequence(Commands.defer(
				() -> {
					Pose2d scoringPose = FieldLayout.getProcessorScoringPose();

					return new FollowSyncedPIDToPose(scoringPose, Level.PROCESSOR_ALGAE, true);
				},
				Set.of(Drive.mInstance)));
	}

	public Command stowAlgaeWhenReady() {
		return Commands.sequence(waitUntilFarFromReef(), algaeStow());
	}

	public Command waitUntilFarFromReef() {
		return Commands.waitUntil(() -> Util.getDistanceFromReef().gte(SuperstructureConstants.kAlgaeStowReefDistance)
				|| Math.abs(Util.getAngleFromReef().getDegrees()) > 90.0);
	}

	public Command reefIntakeAtRightLevel() {
		Set<Subsystem> requirements = Set.of(Elevator.mInstance, Pivot.mInstance, EndEffector.mInstance);
		Supplier<Command> supplier = () -> {
			return reefAlgaeIntake(getTargetingL3Algae());
		};
		return Commands.sequence(
						Commands.runOnce(() -> algaeAligning = true),
						Commands.defer(supplier, requirements),
						Commands.runOnce(() -> algaeAligning = false))
				.handleInterrupt(() -> algaeAligning = false);
	}

	public Command goToReefIntakeReadyPose() {
		return Commands.defer(() -> goToReefIntakePose(targetingFace), Set.of(Drive.mInstance));
	}

	public Command goToReefIntakeReadyPose(Face face) {
		Supplier<Command> supplier = () -> {
			Pose2d pose = FieldLayout.handleAllianceFlip(
					FieldLayout.getAlgaeReefIntakePose(face), RobotConstants.isRedAlliance);
			Level level = Level.ALGAE_READY;
			return new PIDToPoseCommand(pose, level);
		};
		return Commands.defer(supplier, Set.of(Drive.mInstance));
	}

	public Command goToReefIntakePose() {
		return Commands.defer(() -> goToReefIntakePose(targetingFace), Set.of(Drive.mInstance));
	}

	public Command goToReefIntakePose(Face face) {
		Supplier<Command> supplier = () -> {
			Pose2d pose = FieldLayout.handleAllianceFlip(
					FieldLayout.getAlgaeReefIntakePose(face), RobotConstants.isRedAlliance);
			Level level = getTargetingL3Algae() ? Level.L3_ALGAE : Level.L2_ALGAE;
			return new FollowSyncedTagPIDToPose(pose, level);
		};
		return Commands.defer(supplier, Set.of(Drive.mInstance));
	}

	public Command elevatorAtRightPointForAlgaeIntakeFromReef() {
		return Commands.defer(
				() -> {
					BooleanSupplier test = () -> Elevator.mInstance.nearPosition(ElevatorConstants.converter.toAngle(
							targetingL3ReefIntake
									? ElevatorConstants.kL3AlgaePosition
									: ElevatorConstants.kL2AlgaePosition));
					return Commands.waitUntil(test);
				},
				Util.getEmptySubsystemSet());
	}

	public Command alignToNet() {
		return new DeferredCommand(() -> new FollowXLineSwerveRequestCommand(), Set.of(Drive.mInstance))
				.withName("Align to Net");
	}

	public Command scoreCoralWhenReady(Level level) {
		if (level.equals(Level.L1)) {
			return Commands.sequence(waitUntilDriveReadyToScoreAndStable(), softCoralScore())
					.withName("Score Coral When Ready");
		} else {
			return Commands.sequence(waitUntilDriveReadyToScoreAndStable(), coralScore(level))
					.withName("Score Coral When Ready");
		}
	}

	public Command waitToStartScoreSequence() {
		return Commands.waitUntil(() -> endEffectorCoralBreak.getDebounced() && readyToRaiseElevator)
				.withName("Wait To Start Score Sequence");
	}

	public Command waitToStartAlgaeScoreSequence() {
		return Commands.sequence(Commands.waitUntil(() -> endEffectorAlgaeBreak.getDebounced() && readyToRaiseElevator))
				.withName("Wait to Start Algae Score Sequence");
	}

	public Command waitUntilDriveReadyToScoreAndStable() {
		return Commands.waitUntil(() -> getDriveReady() && Drive.mInstance.getStable() && pivotStable());
	}

	public Command setHasAlgaeCommand(boolean hasAlgae) {
		return Commands.runOnce(() -> setHasAlgae(hasAlgae));
	}

	public static enum State {
		TUCK,
		SPIT,
		GROUND_CORAL,
		GROUND_ALGAE,
		STATION,
		LOLLIPOP_ALGAE,
		HOLD_CORAL,
		HOLD_ALGAE,
		L1_CORAL,
		L2_CORAL,
		L3_CORAL,
		L4_CORAL,
		L2_ALGAE,
		L3_ALGAE,
		PROCESSOR,
		NET,
		GULP,
		GROUND_CORAL_WITH_ALGAE;
	}

	public Command setState(State state) {
		return Commands.runOnce(() -> this.state = state);
	}

	public State getState() {
		return state;
	}

	public Command stationIntakePrep() {
		return Commands.sequence(
						MotionPlanner.safePivotAndElevatorToPosition(Pivot.CORAL_STATION, Elevator.CORAL_STATION),
						AlgaeDeploy.mInstance.setpointCommand(AlgaeDeploy.STOW),
						EndEffector.mInstance.setpointCommand(EndEffector.STATION_INTAKE),
						setState(State.STATION))
				.withName("Station Intake Prep");
	}

	public Command stationIntake() {
		return Commands.sequence(
						endEffectorVelocityDip.stateWait(true),
						EndEffector.mInstance.setpointCommand(EndEffector.CORAL_HOLD))
				.handleInterrupt(() -> EndEffector.mInstance.applySetpoint(EndEffector.CORAL_HOLD))
				.withName("Station Intake");
	}

	public Command stationIntakeToHold() {
		return Commands.sequence(stationIntakePrep(), stationIntake(), waitUntilClearFromStation(), stowCoralHold())
				.withName("Station Intake To Hold");
	}

	public Command waitUntilClearFromStation() {
		return Commands.waitUntil(() -> Util.getDistanceFromReef().lte(Units.Meters.of(4.0)));
	}

	public Command waitUntilDistanceFromReef(Distance d) {
		return Commands.waitUntil(() -> Util.getDistanceFromReef().lte(d));
	}

	public Command waitToGulp() {
		return Commands.race(
				Commands.sequence(Commands.waitSeconds(1.0), coralRollersCurrentSpike.stateWaitWithDebounce(true)),
				endEffectorCoralBreak.stateWaitWithDebounce(true),
				new WaitUntilCommand(() -> forceGulp));
	}

	public boolean getForceGulp() {
		return forceGulp;
	}

	public boolean getHasAlgae() {
		return hasAlgae;
	}

	public boolean closeToNetLine() {
		boolean useRedAlliance = Drive.mInstance
						.getPose()
						.getMeasureX()
						.minus(FieldLayout.getNetScoreLine(true))
						.abs(BaseUnits.DistanceUnit)
				< Drive.mInstance
						.getPose()
						.getMeasureX()
						.minus(FieldLayout.getNetScoreLine(false))
						.abs(BaseUnits.DistanceUnit);

		return DriveConstants.distanceToRaiseElevatorNet.gte(Units.Meters.of(FieldLayout.getNetScoreLine(useRedAlliance)
				.minus(Units.Meters.of(Drive.mInstance.getPose().getX()))
				.abs(BaseUnits.DistanceUnit)));
	}

	public boolean closeToNetLineInAuto() {
		boolean useRedAlliance = Drive.mInstance
						.getPose()
						.getMeasureX()
						.minus(FieldLayout.getNetScoreLine(true))
						.abs(BaseUnits.DistanceUnit)
				< Drive.mInstance
						.getPose()
						.getMeasureX()
						.minus(FieldLayout.getNetScoreLine(false))
						.abs(BaseUnits.DistanceUnit);

		return DriveConstants.distanceToRaiseElevatorNet
				.times(0.5)
				.gte(Units.Meters.of(FieldLayout.getNetScoreLine(useRedAlliance)
						.minus(Units.Meters.of(Drive.mInstance.getPose().getX()))
						.abs(BaseUnits.DistanceUnit)));
	}

	public boolean elevatorAtOrBelowL2() {
		Angle l2PosAsAngle = ElevatorConstants.converter.toAngle(ElevatorConstants.kL2ScoringHeight);
		return Elevator.mInstance.nearPosition(l2PosAsAngle)
				|| Elevator.mInstance.getPosition().lte(l2PosAsAngle);
	}

	/**
	 * Checks if the robot is clear from BOTH processors
	 */
	public boolean getClearFromProcessor() {
		return getClearFromProcessor(true) && getClearFromProcessor(false);
	}

	/**
	 * Checks if the robot is clear from A SPECIFIED processor
	 *
	 * @param isBlueProc true if checking blue, false if checking red
	 */
	public boolean getClearFromProcessor(boolean isBlueProc) {
		Pose2d flippedProcessorPose = FieldLayout.handleAllianceFlip(FieldLayout.blueProcessorPose, isBlueProc);
		Pose2d drivePose = Drive.mInstance.getPose();
		return Math.abs(drivePose
								.getTranslation()
								.minus(flippedProcessorPose.getTranslation())
								.getAngle()
								.minus(drivePose.getRotation())
								.getDegrees())
						> 120.0
				|| flippedProcessorPose
								.getTranslation()
								.getDistance(Drive.mInstance.getPose().getTranslation())
						> 1.0;
	}

	public Branch getTargetingBranch() {
		return targetingBranch;
	}

	public Face getTargetingFace() {
		return targetingFace;
	}

	public boolean getEndEffectorCoralBreak() {
		return endEffectorCoralBreak.get();
	}

	public void setDriveReady(boolean valToSet) {
		driveReady = valToSet;
	}

	public void setReadyToRaiseElevator(boolean valToSet) {
		readyToRaiseElevator = valToSet;
	}

	public void setSuperstructureDone(boolean valToSet) {
		superstructureDone = valToSet;
	}

	public boolean getDriveReady() {
		return driveReady;
	}

	public boolean pivotStable() {
		return Pivot.mInstance.getVelocity().abs(Units.DegreesPerSecond)
				< SuperstructureConstants.kPivotStableThresholdVelocity.in(Units.DegreesPerSecond);
	}

	public boolean getSuperstructureDone() {
		return superstructureDone;
	}

	public boolean getAlgaeAligning() {
		return algaeAligning;
	}

	public boolean getEndEffectorAlgae() {
		return endEffectorAlgaeBreak.get();
	}

	public boolean getTargetingL3Algae() {
		return targetingL3ReefIntake;
	}

	public void setHasAlgae(boolean has) {
		hasAlgae = has;
	}

	public void setPathFollowing(boolean isFollowing) {
		isPathFollowing = isFollowing;
	}
}
