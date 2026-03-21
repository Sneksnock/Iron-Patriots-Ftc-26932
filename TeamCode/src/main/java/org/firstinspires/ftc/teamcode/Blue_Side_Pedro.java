package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.teamcode.shooter.ie;
import static org.firstinspires.ftc.teamcode.shooter.ieP;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@Autonomous(name = "Blue Pedro")
public class Blue_Side_Pedro extends OpMode {

    /// ---------- HARDWARE ----------
    private shooter shooter;
    private Follower follower;

    /// ---------- TIMERS ----------
    public Timer opModeTimer, waitTimer;
    private ElapsedTime poseTimer;

    /// ---------- STATE ----------
    private int pathState = 0;
    private static final double POSE_TIMEOUT = 4.0;

    /// ---------- POSE DATA ----------
    private final Pose startPose = new Pose(33.916, 126.968, 2.4196);
    private final Pose scorePose = new Pose(85, 133, 3.1);

    // Line 1
    private final Pose line1PrePose = new Pose(54.25, 90.5, 3.07);
    private final Pose intakeLine1Pose = new Pose(29.75, 83.5, 3.07);

    // Line 2
    private final Pose line2PrePose = new Pose(54.25, 67.5, 3.07);
    private final Pose intakeLine2Pose = new Pose(23, 62, 3.07);

    // Line 3
    private final Pose line3PrePose = new Pose(47.5, 45.5, 3.07);
    private final Pose intakeLine3Pose = new Pose(20.5, 34.9, 3.07);

    // Lever
    private final Pose leverPrePose = new Pose(25, 60, 2.56);
    private final Pose leverPose = new Pose(19.0, 62.082, 2.56);

    /// ---------- PATHS ----------
    private PathChain score1;
    private PathChain line1PrePath;
    private PathChain intakeLine1Path;
    private PathChain score2;

    private PathChain line2PrePath;
    private PathChain intakeLine2Path;
    private PathChain score3;

    private PathChain line3PrePath;
    private PathChain intakeLine3Path;
    private PathChain score4;

    private PathChain leverPrePath;
    private PathChain leverPath;
    private PathChain leverScorePath;

    @Override
    public void init() {
        shooter = new shooter();
        opModeTimer = new Timer();
        waitTimer = new Timer();
        poseTimer = new ElapsedTime();

        opModeTimer.resetTimer();
        waitTimer.resetTimer();
        poseTimer.reset();

        shooter.init(hardwareMap);
        follower = Constants.createFollower(hardwareMap);

        buildPaths();
        follower.setStartingPose(startPose);
        follower.setMaxPower(1.0);
    }

    @Override
    public void loop() {
        follower.update();
        shooter.update();

        autonomousPathUpdate();

        /// ---------- TELEMETRY ----------
        telemetry.addData("Path State", pathState);
        telemetry.addData("Shooter State", shooter.getState());
        telemetry.addData("Follower Busy", follower.isBusy());
        telemetry.addData("Velocity", follower.getVelocity());
        telemetry.addData("Shots Remaining", shooter.ShotsRemaining);
        telemetry.addData("Pose Timer", poseTimer.seconds());
        telemetry.update();
    }

    private void startPath(PathChain path, boolean holdEnd) {
        follower.followPath(path, holdEnd);
        poseTimer.reset();
    }

    private boolean poseDone() {
        return !follower.isBusy() || poseTimer.seconds() >= POSE_TIMEOUT;
    }

    /// ---------- PATHING ----------
    public void autonomousPathUpdate() {
        switch (pathState) {

            // Drive to first score
            case 0:
                follower.setMaxPower(1.0);
                startPath(score1, true);
                pathState = 1;
                break;

            // Wait until at first score
            case 1:
                if (poseDone()) {
                    pathState = 2;
                }
                break;

            // Shoot first volley
            case 2:
                shooter.shoot();
                pathState = 3;
                break;

            // Wait until first volley finishes
            case 3:
                if (shooter.ShotsRemaining <= 0) {
                    pathState = 4;
                }
                break;

            // Drive to Line 1 pre-intake
            case 4:
                ie.setPower(ieP);
                follower.setMaxPower(1.0);
                startPath(line1PrePath, true);
                pathState = 5;
                break;

            // Wait until at Line 1 pre-intake
            case 5:
                ie.setPower(ieP);
                if (poseDone()) {
                    pathState = 6;
                }
                break;

            // Drive through Line 1 intake
            case 6:
                ie.setPower(ieP);
                follower.setMaxPower(0.75);
                startPath(intakeLine1Path, true);
                pathState = 7;
                break;

            // Wait until Line 1 intake finishes
            case 7:
                ie.setPower(ieP);
                if (poseDone()) {
                    follower.setMaxPower(1.0);
                    pathState = 8;
                }
                break;

            // Return to score second volley
            case 8:
                ie.setPower(ieP);
                follower.setMaxPower(1.0);
                startPath(score2, true);
                pathState = 9;
                break;

            // Wait until back at score
            case 9:
                ie.setPower(ieP);
                if (poseDone()) {
                    pathState = 10;
                }
                break;

            // Shoot second volley
            case 10:
                shooter.shoot();
                pathState = 11;
                break;

            // Wait until second volley finishes
            case 11:
                if (shooter.ShotsRemaining <= 0) {
                    pathState = 12;
                }
                break;

            // Drive to Line 2 pre-intake
            case 12:
                ie.setPower(ieP);
                follower.setMaxPower(1.0);
                startPath(line2PrePath, true);
                pathState = 13;
                break;

            // Wait until at Line 2 pre-intake
            case 13:
                ie.setPower(ieP);
                if (poseDone()) {
                    pathState = 14;
                }
                break;

            // Drive through Line 2 intake
            case 14:
                ie.setPower(ieP);
                follower.setMaxPower(0.75);
                startPath(intakeLine2Path, true);
                pathState = 15;
                break;

            // Wait until Line 2 intake finishes
            case 15:
                ie.setPower(ieP);
                if (poseDone()) {
                    follower.setMaxPower(1.0);
                    pathState = 16;
                }
                break;

            // Return to score third volley
            case 16:
                ie.setPower(ieP);
                follower.setMaxPower(1.0);
                startPath(score3, true);
                pathState = 17;
                break;

            // Wait until back at score
            case 17:
                ie.setPower(ieP);
                if (poseDone()) {
                    pathState = 18;
                }
                break;

            // Shoot third volley
            case 18:
                shooter.shoot();
                pathState = 19;
                break;

            // Wait until third volley finishes
            case 19:
                if (shooter.ShotsRemaining <= 0) {
                    pathState = 20;
                }
                break;

            // Drive to Line 3 pre-intake
            case 20:
                ie.setPower(ieP);
                follower.setMaxPower(1.0);
                startPath(line3PrePath, true);
                pathState = 21;
                break;

            // Wait until at Line 3 pre-intake
            case 21:
                ie.setPower(ieP);
                if (poseDone()) {
                    pathState = 22;
                }
                break;

            // Drive through Line 3 intake
            case 22:
                ie.setPower(ieP);
                follower.setMaxPower(0.75);
                startPath(intakeLine3Path, true);
                pathState = 23;
                break;

            // Wait until Line 3 intake finishes
            case 23:
                ie.setPower(ieP);
                if (poseDone()) {
                    follower.setMaxPower(1.0);
                    pathState = 24;
                }
                break;

            // Return to score fourth volley
            case 24:
                ie.setPower(ieP);
                follower.setMaxPower(1.0);
                startPath(score4, true);
                pathState = 25;
                break;

            // Wait until back at score
            case 25:
                ie.setPower(ieP);
                if (poseDone()) {
                    pathState = 26;
                }
                break;

            // Shoot fourth volley
            case 26:
                shooter.shoot();
                pathState = 27;
                break;

            // Wait until fourth volley finishes
            case 27:
                if (shooter.ShotsRemaining <= 0) {
                    pathState = 28;
                }
                break;

            // Done
            case 28:
                ie.setPower(0);
                follower.setMaxPower(1.0);
                break;
        }
    }

    public void buildPaths() {

        /// ---------- SCORE PATH ----------
        score1 = follower.pathBuilder()
                .addPath(new BezierLine(startPose, scorePose))
                .setLinearHeadingInterpolation(startPose.getHeading(), scorePose.getHeading())
                .build();

        /// ---------- LINE 1 PATHING ----------
        line1PrePath = follower.pathBuilder()
                .addPath(new BezierLine(scorePose, line1PrePose))
                .setLinearHeadingInterpolation(scorePose.getHeading(), line1PrePose.getHeading())
                .build();

        intakeLine1Path = follower.pathBuilder()
                .addPath(new BezierLine(line1PrePose, intakeLine1Pose))
                .setLinearHeadingInterpolation(line1PrePose.getHeading(), intakeLine1Pose.getHeading())
                .build();

        score2 = follower.pathBuilder()
                .addPath(new BezierLine(intakeLine1Pose, scorePose))
                .setLinearHeadingInterpolation(intakeLine1Pose.getHeading(), scorePose.getHeading())
                .build();

        /// ---------- LINE 2 PATHING ----------
        line2PrePath = follower.pathBuilder()
                .addPath(new BezierLine(scorePose, line2PrePose))
                .setLinearHeadingInterpolation(scorePose.getHeading(), line2PrePose.getHeading())
                .build();

        intakeLine2Path = follower.pathBuilder()
                .addPath(new BezierLine(line2PrePose, intakeLine2Pose))
                .setLinearHeadingInterpolation(line2PrePose.getHeading(), intakeLine2Pose.getHeading())
                .build();

        score3 = follower.pathBuilder()
                .addPath(new BezierLine(intakeLine2Pose, scorePose))
                .setLinearHeadingInterpolation(intakeLine2Pose.getHeading(), scorePose.getHeading())
                .build();

        /// ---------- LINE 3 PATHING ----------
        line3PrePath = follower.pathBuilder()
                .addPath(new BezierLine(scorePose, line3PrePose))
                .setLinearHeadingInterpolation(scorePose.getHeading(), line3PrePose.getHeading())
                .build();

        intakeLine3Path = follower.pathBuilder()
                .addPath(new BezierLine(line3PrePose, intakeLine3Pose))
                .setLinearHeadingInterpolation(line3PrePose.getHeading(), intakeLine3Pose.getHeading())
                .build();

        score4 = follower.pathBuilder()
                .addPath(new BezierLine(intakeLine3Pose, scorePose))
                .setLinearHeadingInterpolation(intakeLine3Pose.getHeading(), scorePose.getHeading())
                .build();

        /// ---------- LEVER PATHING ----------
        leverPrePath = follower.pathBuilder()
                .addPath(new BezierLine(scorePose, leverPrePose))
                .setLinearHeadingInterpolation(scorePose.getHeading(), leverPrePose.getHeading())
                .build();

        leverPath = follower.pathBuilder()
                .addPath(new BezierLine(leverPrePose, leverPose))
                .setLinearHeadingInterpolation(leverPrePose.getHeading(), leverPose.getHeading())
                .build();

        leverScorePath = follower.pathBuilder()
                .addPath(new BezierLine(leverPose, scorePose))
                .setLinearHeadingInterpolation(leverPose.getHeading(), scorePose.getHeading())
                .build();
    }
}