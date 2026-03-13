package frc.robot.vision;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;

public class LimelightPoseSelector {

    private final NetworkTable llFront;
    private final NetworkTable llRR;

    public static class VisionResult {
        public final Pose2d pose;
        public final double timestamp;
        public final boolean valid;

        public VisionResult(Pose2d pose, double timestamp, boolean valid) {
            this.pose = pose;
            this.timestamp = timestamp;
            this.valid = valid;
        }
    }

    public LimelightPoseSelector() {
        llFront = NetworkTableInstance.getDefault().getTable("limelight-front");
        llRR = NetworkTableInstance.getDefault().getTable("limelight-rr");
    }

    public VisionResult getBestVisionPose() {
        VisionResult front = readCamera(llFront);
        VisionResult rr = readCamera(llRR);

        // If only one is valid, return it
        if (front.valid && !rr.valid) return front;
        if (rr.valid && !front.valid) return rr;

        // If neither valid, return invalid
        if (!front.valid && !rr.valid) {
            return new VisionResult(null, 0, false);
        }

        // Score both
        double frontScore = scoreCamera(llFront);
        double rrScore = scoreCamera(llRR);

        return (frontScore >= rrScore) ? front : rr;
    }

    private VisionResult readCamera(NetworkTable table) {
        double tv = table.getEntry("tv").getDouble(0);
        if (tv < 1) return new VisionResult(null, 0, false);

        double[] botpose = table.getEntry("botpose_wpiblue").getDoubleArray(new double[7]);
        if (botpose.length < 7) return new VisionResult(null, 0, false);

        double ambiguity = table.getEntry("poseambiguity").getDouble(1.0);
        if (ambiguity > 0.2) return new VisionResult(null, 0, false);

        double ta = table.getEntry("ta").getDouble(0);
        if (ta < 0.5) return new VisionResult(null, 0, false);

        Pose2d pose = new Pose2d(
            botpose[0],
            botpose[1],
            edu.wpi.first.math.geometry.Rotation2d.fromDegrees(botpose[5])
        );

        double timestamp = edu.wpi.first.wpilibj.Timer.getFPGATimestamp() - botpose[6];

        return new VisionResult(pose, timestamp, true);
    }

    private double scoreCamera(NetworkTable table) {
        double ta = table.getEntry("ta").getDouble(0);
        double ambiguity = table.getEntry("poseambiguity").getDouble(1.0);
        return ta / (1.0 + ambiguity);
    }
}
