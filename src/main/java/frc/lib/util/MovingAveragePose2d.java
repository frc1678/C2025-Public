package frc.lib.util;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import java.util.ArrayList;

/**
 * Helper class for storing and calculating a moving average of the Pose2d class
 */
public class MovingAveragePose2d {
	ArrayList<Pose2d> poses = new ArrayList<Pose2d>();
	private int maxSize;

	public MovingAveragePose2d(int maxSize) {
		this.maxSize = maxSize;
	}

	public synchronized void add(Pose2d pose) {
		poses.add(pose);
		if (poses.size() > maxSize) {
			poses.remove(0);
		}
	}

	public synchronized Pose2d getAverage() {
		double x = 0.0, y = 0.0, t = 0.0;

		for (Pose2d pose : poses) {
			x += pose.getX();
			y += pose.getY();
			t += pose.getRotation().getRadians();
		}

		double size = getSize();
		return new Pose2d(x / size, y / size, Rotation2d.fromRadians(t / size));
	}

	public int getSize() {
		return poses.size();
	}

	public boolean isUnderMaxSize() {
		return getSize() < maxSize;
	}

	public boolean hasUpdate() {
		return getSize() != 0;
	}

	public void clear() {
		poses.clear();
	}
}
