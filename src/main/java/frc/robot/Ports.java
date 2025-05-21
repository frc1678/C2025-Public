package frc.robot;

/**
 * Holds ports IDs and canbuses for superstructure (subsystems that are not drivetrain)
 * Do NOT use ports 0 through 7.
 */
public enum Ports {
	ALGAE_DEPLOY(8, "canivore1"),
	ALGAE_ROLLERS(9, "rio"),
	CORAL_DEPLOY(10, "canivore1"),
	CORAL_ROLLERS(11, "rio"),
	CORAL_INDEXER(12, "canivore1"),
	END_EFFECTOR(13, "canivore1"),
	ELEVATOR_MAIN(14, "canivore1"),
	ELEVATOR_FOLLOWER(15, "canivore1"),
	PIVOT(16, "canivore1"),
	CLIMBER(17, "canivore1"),
	CLIMBER_ROLLERS(18, "canivore1"),
	CANDLE(21, "canivore1"),

	END_EFFECTOR_CORAL_BREAMBREAK(RobotConstants.isComp ? 1 : 8, "RioDigitalIn"),
	END_EFFECTOR_ALGAE_BEAMBREAK(RobotConstants.isComp ? 0 : 7, "RioDigitalIn"),
	INDEXER_BEAMBREAK(RobotConstants.isComp ? 8 : 6, "RioDigitalIn"),

	ENCODER_41T(4, "canivore1"),
	ENCODER_39T(5, "canivore1"),

	PHYSICAL_BUTTON(9, "RioDigitalIn");

	public final int id;
	public final String bus;

	private Ports(int id, String bus) {
		this.id = id;
		this.bus = bus;
	}
}
