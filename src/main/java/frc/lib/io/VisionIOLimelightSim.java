package frc.lib.io;

import frc.lib.util.LimelightHelpers.PoseEstimate;

/**
 * Does nothing, used for simulation.
 */
public class VisionIOLimelightSim extends VisionIOLimelight {
	@Override
	public void update() {}

	@Override
	public void setLatestEstimate(PoseEstimate poseEstimate, int minTagNum) {}
}
