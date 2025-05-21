package frc.lib.io;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.Robot;

public abstract class DetectionIO {
	protected boolean disabled = Robot.isSimulation();

	public void disable(boolean disable) {
		disabled = disable;
	}

	public abstract void update();

	public abstract Pose2d getCoralPose(Translation2d base);

	public boolean getDisabled() {
		return disabled;
	}

	public abstract boolean txComplete(double tx);

	public abstract int coralCount();

	public abstract Translation2d calcDistToCoral(double tx, double ty);

	public abstract void setPipeline(int index);
}
