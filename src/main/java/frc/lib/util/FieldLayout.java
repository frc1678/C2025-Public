package frc.lib.util;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Distance;
import frc.lib.logging.LogUtil;
import frc.lib.util.FieldLayout.Branch.Face;
import frc.robot.RobotConstants;
import frc.robot.autos.AutoConstants.AutoType;
import frc.robot.subsystems.drive.Drive;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;

/**
 * Contains various field dimensions and useful reference points. Dimensions are
 * in meters, and sets
 * of corners start in the lower left moving clockwise. <b>All units in
 * Meters</b> <br>
 * <br>
 *
 * <p>
 * All translations and poses are stored with the origin at the rightmost point
 * on the BLUE
 * ALLIANCE wall.<br>
 * <br>
 * Length refers to the <i>x</i> direction (as described by wpilib) <br>
 * Width refers to the <i>y</i> direction (as described by wpilib)
 */
public class FieldLayout {
	public static Distance kFieldLength = Units.Feet.of(57.0).plus(Units.Inches.of(6.875));
	public static Distance kFieldWidth = Units.Feet.of(26.0).plus(Units.Inches.of(5.0));

	public static AprilTagFieldLayout kAprilTagMap =
			AprilTagFieldLayout.loadField(AprilTagFields.k2025ReefscapeAndyMark);
	public static Distance kAprilTagWidth = Units.Inches.of(6.5);

	public static final Pose2d blueRightStationAlignPose =
			new Pose2d(Units.Meters.of(1.28), Units.Meters.of(0.91), Rotation2d.fromDegrees(-127.5));
	public static final Pose2d blueProcessorPose =
			new Pose2d(Units.Meters.of(5.87), Units.Meters.of(0.394), Rotation2d.fromDegrees(-90.0));
	public static final Pose2d redProcessorPose = handleAllianceFlip(blueProcessorPose, true);
	public static final Translation2d blueReefCenter = new Translation2d(Units.Inches.of(176.75), kFieldWidth.div(2.0));
	public static final Translation2d blueRightStationCenter =
			new Translation2d(Units.Meters.of(0.6), Units.Meters.of(0.38));

	public static final Distance markSeparation = Units.Inches.of(72.0);
	public static final Translation2d blueMidMark = new Translation2d(Units.Inches.of(48.0), kFieldWidth.div(2.0));
	public static final Translation2d blueLeftMark =
			new Translation2d(Units.Inches.of(48.0), blueMidMark.getMeasureY().plus(markSeparation));
	public static final Translation2d blueRightMark =
			new Translation2d(Units.Inches.of(48.0), blueMidMark.getMeasureY().minus(markSeparation));

	public static Pose2d handleAllianceFlip(Pose2d blue_pose, boolean is_red_alliance) {
		if (is_red_alliance) {
			blue_pose = rotateAboutCenter(blue_pose, Rotation2d.k180deg);
		}
		return blue_pose;
	}

	public static Translation2d handleAllianceFlip(Translation2d blue_translation, boolean is_red_alliance) {
		if (is_red_alliance) {
			blue_translation = blue_translation.rotateAround(
					new Translation2d(kFieldLength.div(2.0), kFieldWidth.div(2.0)), Rotation2d.k180deg);
		}
		return blue_translation;
	}

	public static Rotation2d handleAllianceFlip(Rotation2d blue_rotation, boolean is_red_alliance) {
		if (is_red_alliance) {
			blue_rotation = blue_rotation.plus(Rotation2d.k180deg);
		}
		return blue_rotation;
	}

	public static Distance distanceFromAllianceWall(Distance x_coordinate, boolean is_red_alliance) {
		if (is_red_alliance) {
			return kFieldLength.minus(x_coordinate);
		}
		return x_coordinate;
	}

	public static Translation2d mirrorAboutX(Translation2d t, Distance xValue) {
		return new Translation2d(xValue.in(Units.Meters) + (xValue.in(Units.Meters) - t.getX()), t.getY());
	}

	public static Translation2d mirrorAboutY(Translation2d t, Distance yValue) {
		return new Translation2d(t.getX(), yValue.in(Units.Meters) + (yValue.in(Units.Meters) - t.getY()));
	}

	public static Rotation2d mirrorAboutX(Rotation2d r) {
		return new Rotation2d(-r.getCos(), r.getSin());
	}

	public static Rotation2d mirrorAboutY(Rotation2d r) {
		return new Rotation2d(r.getCos(), -r.getSin());
	}

	public static Pose2d mirrorAboutX(Pose2d p, Distance xValue) {
		return new Pose2d(mirrorAboutX(p.getTranslation(), xValue), mirrorAboutX(p.getRotation()));
	}

	public static Pose2d mirrorAboutY(Pose2d p, Distance yValue) {
		return new Pose2d(mirrorAboutY(p.getTranslation(), yValue), mirrorAboutY(p.getRotation()));
	}

	public static Pose2d rotateAboutPose(Pose2d startPose, Translation2d point, Rotation2d rotation) {
		return new Pose2d(
				startPose.getTranslation().rotateAround(point, rotation),
				startPose.getRotation().plus(rotation));
	}

	public static Pose2d rotateAboutCenter(Pose2d startPose, Rotation2d rotation) {
		return rotateAboutPose(startPose, new Translation2d(kFieldLength.div(2.0), kFieldWidth.div(2.0)), rotation);
	}

	public static Rotation2d getBranchFaceRotation(Branch branch) {
		return branch.getKey().face.rotation;
	}

	public static Rotation2d getFaceRotation(Face face) {
		return face.rotation;
	}

	public static Pose2d getCoralScoringPose(Branch branch) {
		Branch.BranchKey key = branch.getKey();
		Rotation2d branchFaceRotation = getBranchFaceRotation(branch);
		Transform2d centerOfReefToScoringLocation = new Transform2d(
				coralInOutDistance,
				key.isLeft ? coralLeftRightDistance : coralLeftRightDistance.times(-1.0),
				new Rotation2d());
		return new Pose2d(blueReefCenter, branchFaceRotation).transformBy(centerOfReefToScoringLocation);
	}

	public static Pose2d getAlgaeReefIntakePose(Face face) {
		Rotation2d faceRotation = getFaceRotation(face);
		Transform2d centerOfReefToReefIntakeLocation =
				new Transform2d(algaeInOutDistance, Units.Inches.of(0.0), new Rotation2d());
		return new Pose2d(blueReefCenter, faceRotation).transformBy(centerOfReefToReefIntakeLocation);
	}

	public static enum Branch {
		A(new BranchKey(true, Face.NEAR_CENTER)),
		B(new BranchKey(false, Face.NEAR_CENTER)),
		C(new BranchKey(true, Face.NEAR_RIGHT)),
		D(new BranchKey(false, Face.NEAR_RIGHT)),
		E(new BranchKey(true, Face.FAR_RIGHT)),
		F(new BranchKey(false, Face.FAR_RIGHT)),
		G(new BranchKey(true, Face.FAR_CENTER)),
		H(new BranchKey(false, Face.FAR_CENTER)),
		I(new BranchKey(true, Face.FAR_LEFT)),
		J(new BranchKey(false, Face.FAR_LEFT)),
		K(new BranchKey(true, Face.NEAR_LEFT)),
		L(new BranchKey(false, Face.NEAR_LEFT));

		private final BranchKey key;

		public static enum Face {
			FAR_LEFT(Rotation2d.fromDegrees(240.0)), // IJ
			NEAR_LEFT(Rotation2d.fromDegrees(300.0)), // KL
			FAR_RIGHT(Rotation2d.fromDegrees(120.0)), // EF
			NEAR_RIGHT(Rotation2d.fromDegrees(60.0)), // CD
			FAR_CENTER(Rotation2d.fromDegrees(180.0)), // GH
			NEAR_CENTER(Rotation2d.fromDegrees(0.0)); // AB

			private final Rotation2d rotation;

			private Face(Rotation2d rotation) {
				this.rotation = rotation;
			}
		}

		public static Map<Pose2d, Face> blueFaceMap = new HashMap<>();

		static {
			blueFaceMap.put(kAprilTagMap.getTagPose(17).get().toPose2d(), Face.NEAR_RIGHT);
			blueFaceMap.put(kAprilTagMap.getTagPose(18).get().toPose2d(), Face.NEAR_CENTER);
			blueFaceMap.put(kAprilTagMap.getTagPose(19).get().toPose2d(), Face.NEAR_LEFT);
			blueFaceMap.put(kAprilTagMap.getTagPose(20).get().toPose2d(), Face.FAR_LEFT);
			blueFaceMap.put(kAprilTagMap.getTagPose(21).get().toPose2d(), Face.FAR_CENTER);
			blueFaceMap.put(kAprilTagMap.getTagPose(22).get().toPose2d(), Face.FAR_RIGHT);
		}

		public static Face getClosestFace(Pose2d pose, boolean isRedAlliance) {
			Pose2d bluePose = handleAllianceFlip(pose, isRedAlliance);
			Pose2d nearestPose = bluePose.nearest(new ArrayList<>(blueFaceMap.keySet()));
			return blueFaceMap.get(nearestPose);
		}

		public static record BranchKey(boolean isLeft, Face face) {}

		public static Map<BranchKey, Branch> branchMap = new HashMap<>();

		static {
			branchMap.put(A.getKey(), A);
			branchMap.put(B.getKey(), B);
			branchMap.put(C.getKey(), C);
			branchMap.put(D.getKey(), D);
			branchMap.put(E.getKey(), E);
			branchMap.put(F.getKey(), F);
			branchMap.put(G.getKey(), G);
			branchMap.put(H.getKey(), H);
			branchMap.put(I.getKey(), I);
			branchMap.put(J.getKey(), J);
			branchMap.put(K.getKey(), K);
			branchMap.put(L.getKey(), L);
		}

		public static Map<Pose2d, Branch> leftBranches = new HashMap<>();

		static {
			leftBranches.put(getCoralScoringPose(A), A);
			leftBranches.put(getCoralScoringPose(C), C);
			leftBranches.put(getCoralScoringPose(E), E);
			leftBranches.put(getCoralScoringPose(G), G);
			leftBranches.put(getCoralScoringPose(I), I);
			leftBranches.put(getCoralScoringPose(K), K);
		}

		public static Map<Pose2d, Branch> rightBranches = new HashMap<>();

		static {
			rightBranches.put(getCoralScoringPose(B), B);
			rightBranches.put(getCoralScoringPose(D), D);
			rightBranches.put(getCoralScoringPose(F), F);
			rightBranches.put(getCoralScoringPose(H), H);
			rightBranches.put(getCoralScoringPose(J), J);
			rightBranches.put(getCoralScoringPose(L), L);
		}

		public static Map<Pose2d, Branch> branches = new HashMap<>();

		static {
			branches.put(getCoralScoringPose(A), A);
			branches.put(getCoralScoringPose(B), B);
			branches.put(getCoralScoringPose(C), C);
			branches.put(getCoralScoringPose(D), D);
			branches.put(getCoralScoringPose(E), E);
			branches.put(getCoralScoringPose(F), F);
			branches.put(getCoralScoringPose(G), G);
			branches.put(getCoralScoringPose(H), H);
			branches.put(getCoralScoringPose(I), I);
			branches.put(getCoralScoringPose(J), J);
			branches.put(getCoralScoringPose(K), K);
			branches.put(getCoralScoringPose(L), L);
		}

		private Branch(BranchKey key) {
			this.key = key;
		}

		public BranchKey getKey() {
			return key;
		}

		public static Branch getWithParams(boolean isLeft, Face face) {
			return branchMap.get(new BranchKey(isLeft, face));
		}

		public static Branch getClosestBranch(Pose2d referencePose, boolean isRedAlliance) {
			Pose2d bluePose = handleAllianceFlip(referencePose, isRedAlliance);
			Map<Pose2d, Branch> branchesMap = branches;
			Pose2d nearestPose = bluePose.nearest(new ArrayList<>(branchesMap.keySet()));
			return branchesMap.get(nearestPose);
		}

		public static Branch getClosestLeftRightBranch(boolean isLeft, Pose2d referencePose, boolean isRedAlliance) {
			Pose2d bluePose = handleAllianceFlip(referencePose, isRedAlliance);
			Map<Pose2d, Branch> branchesMap = isLeft ? leftBranches : rightBranches;
			Pose2d nearestPose = bluePose.nearest(new ArrayList<>(branchesMap.keySet()));
			return branchesMap.get(nearestPose);
		}
	}

	public static enum Level {
		L1,
		L2,
		L2_ALGAE,
		L3,
		L3_ALGAE,
		L4,
		ALGAE_READY,
		PROCESSOR_ALGAE,
		NET
	}

	public static final Distance coralInOutDistance = Units.Inches.of(-30.693);
	public static final Distance coralLeftRightDistance = Units.Inches.of(6.469);

	public static final Distance algaeInOutDistance = Units.Inches.of(-32.0);

	public static final Distance blueRightStationCornerNoX = Units.Meters.of(1.6);
	public static final Distance blueRightStationCornerNoY = Units.Meters.of(0.85);

	public static final Distance fieldCrop = Units.Feet.of(-0.5);

	public static boolean outsideField(Pose2d pose) {
		return pose.getMeasureX().lte(blueRightStationCornerNoX)
						&& pose.getMeasureY().lte(blueRightStationCornerNoY)
				|| pose.getMeasureX().gte(FieldLayout.kFieldLength.minus(blueRightStationCornerNoX))
						&& pose.getMeasureY().lte(blueRightStationCornerNoY)
				|| pose.getMeasureX().lte(blueRightStationCornerNoX)
						&& pose.getMeasureY().gte(FieldLayout.kFieldWidth.minus(blueRightStationCornerNoY))
				|| pose.getMeasureX().gte(FieldLayout.kFieldLength.minus(blueRightStationCornerNoX))
						&& pose.getMeasureY().gte(FieldLayout.kFieldWidth.minus(blueRightStationCornerNoY))
				|| pose.getMeasureX().gte(FieldLayout.kFieldLength.minus(fieldCrop))
				|| pose.getMeasureX().lte(Units.Meters.of(0.0).plus(fieldCrop))
				|| pose.getMeasureY().gte(FieldLayout.kFieldWidth.minus(fieldCrop))
				|| pose.getMeasureY().lte(Units.Meters.of(0.0).plus(fieldCrop));
	}

	public static Pose2d getProcessorScoringPose() {
		return Drive.mInstance.getPose().nearest(List.of(blueProcessorPose, redProcessorPose));
	}

	public static boolean nearMark(Pose2d pose) {
		return Util.epsilonEquals(
						pose.getTranslation(),
						FieldLayout.handleAllianceFlip(blueLeftMark, RobotConstants.isRedAlliance),
						Units.Feet.of(3.0))
				|| Util.epsilonEquals(
						pose.getTranslation(),
						FieldLayout.handleAllianceFlip(blueMidMark, RobotConstants.isRedAlliance),
						Units.Feet.of(3.0))
				|| Util.epsilonEquals(
						pose.getTranslation(),
						FieldLayout.handleAllianceFlip(blueRightMark, RobotConstants.isRedAlliance),
						Units.Feet.of(3.0));
	}

	public static final Distance getNetScoreLine(boolean isRedAlliance) {
		Distance midline = kFieldLength.div(2.0);
		Distance distanceFromMidline =
				isRedAlliance ? distanceFromMidlineNetScore : distanceFromMidlineNetScore.unaryMinus();
		return midline.plus(distanceFromMidline);
	}

	public static final Distance getFieldWidthMidline() {
		return kFieldWidth.div(2.0);
	}

	public static final Rotation2d getNetScoreAngle(boolean isRedAlliance) {
		return isRedAlliance ? netScoreAngle.plus(Rotation2d.k180deg) : netScoreAngle;
	}

	public static final Distance distanceFromMidlineNetScore = Units.Inches.of(34.0);
	public static final Rotation2d netScoreAngle = Rotation2d.fromDegrees(20.0);

	public static boolean getIsInZone(Translation2d target, AutoType side) {
		ArrayList<Translation2d> zone;

		if (RobotConstants.isRedAlliance && side == AutoType.LEFT) {
			zone = AutoTargetZones.redLeftDetectionTargetZone; // red detection ilkj
		} else if (RobotConstants.isRedAlliance && side == AutoType.RIGHT) {
			zone = AutoTargetZones.redRightDetectionTargetZone; // red detection fcde
		} else if (!RobotConstants.isRedAlliance && side == AutoType.LEFT) {
			zone = AutoTargetZones.blueLeftDetectionTargetZone; // blue detection ilkj
		} else {
			zone = AutoTargetZones.blueRightDetectionTargetZone; // blue detection fcde
		}

		for (int i = 0; i < zone.size(); i++) {
			LogUtil.recordTranslation2d("Zone" + i, zone.get(i));
		}

		return PointInPolygon.pointInPolygon(target, zone);
	}

	public static class AutoTargetZones {
		public static Translation2d station =
				new Translation2d(0.448 + fieldCrop.in(Units.Meters), fieldCrop.in(Units.Meters));
		public static Translation2d xLineBot =
				new Translation2d(getCoralScoringPose(Branch.F).getX(), fieldCrop.in(Units.Meters));
		public static Translation2d xLineTop =
				new Translation2d(getCoralScoringPose(Branch.F).getX(), 2.707);
		public static Translation2d deCorner = new Translation2d(4.507, 2.707);
		public static Translation2d bcCorner = new Translation2d(3.312, 3.366);
		public static Translation2d stationWall = new Translation2d(0.448 + fieldCrop.in(Units.Meters), 0.914);

		public static ArrayList<Translation2d> blueRightDetectionTargetZone = new ArrayList<Translation2d>();

		static {
			blueRightDetectionTargetZone.add(station);
			blueRightDetectionTargetZone.add(xLineBot);
			blueRightDetectionTargetZone.add(xLineTop);
			blueRightDetectionTargetZone.add(deCorner);
			blueRightDetectionTargetZone.add(bcCorner);
			blueRightDetectionTargetZone.add(stationWall);
		}

		public static ArrayList<Translation2d> blueLeftDetectionTargetZone = new ArrayList<Translation2d>();

		static {
			for (Translation2d point : blueRightDetectionTargetZone) {
				blueLeftDetectionTargetZone.add(mirrorAboutY(point, kFieldWidth.div(2.0)));
			}
		}

		public static ArrayList<Translation2d> redRightDetectionTargetZone = new ArrayList<Translation2d>();

		static {
			for (Translation2d point : blueLeftDetectionTargetZone) {
				redRightDetectionTargetZone.add(mirrorAboutX(point, kFieldLength.div(2.0)));
			}
		}

		public static ArrayList<Translation2d> redLeftDetectionTargetZone = new ArrayList<Translation2d>();

		static {
			for (Translation2d point : blueRightDetectionTargetZone) {
				redLeftDetectionTargetZone.add(mirrorAboutX(point, kFieldLength.div(2.0)));
			}
		}
	}
}
