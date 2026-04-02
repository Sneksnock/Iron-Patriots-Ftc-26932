package org.firstinspires.ftc.teamcode.pedroPathing;
import com.pedropathing.geometry.Pose;

public class MatchPoseMemory {

    private static boolean hasPose = false;
    private static double x = 0.0;
    private static double y = 0.0;
    private static double heading = 0.0;

    public static void save(Pose pose) {
        if (pose == null) return;
        x = pose.getX();
        y = pose.getY();
        heading = pose.getHeading();
        hasPose = true;
    }

    public static boolean hasPose() {
        return hasPose;
    }

    public static Pose getPoseOrDefault(Pose fallback) {
        if (hasPose) {
            return new Pose(x, y, heading);
        }
        return fallback;
    }

    public static void clear() {
        hasPose = false;
        x = 0.0;
        y = 0.0;
        heading = 0.0;
    }
}
