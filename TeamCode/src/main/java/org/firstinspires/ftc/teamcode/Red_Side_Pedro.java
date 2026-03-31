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

@Autonomous(name = "Red Side", group = "Examples")
public class Red_Side_Pedro extends OpMode {

    /// ---------- HARDWARE ----------
    private Follower follower;
    private shooter shooter;

    /// ---------- TIMERS ----------
    private Timer opModeTimer, waitTimer;
    private ElapsedTime poseTimer;

    /// ---------- STATE ----------
    private int pathState = 0;
    private static final double POSE_TIMEOUT = 4.0;

    /// ---------- POSE DATA ----------
    private final Pose startPose = new Pose(124.311, 123.794, 0.739);
    private final Pose scorePose = new Pose(86, 129.5, 0.17);

    // Line 1
    private final Pose line1PrePose = new Pose(102.5, 90.5, 0.0);
    private final Pose intakeLine1Pose = new Pose(128.0, 83.5, 0.02);

    // Line 2
    private final Pose line2PrePose = new Pose(102.5, 67.5, 0.06);
    private final Pose intakeLine2Pose = new Pose(138, 61.0, 0.01);

    // Line 3
    private final Pose line3PrePose = new Pose(105, 45.4, 0);
    private final Pose intakeLine3Pose = new Pose(137, 34.9, 0);

    // Lever
    private final Pose leverPrePose = new Pose(139, 60, 0.03);
    private final Pose leverPose = new Pose(108, 66, 2.56);

    /// ---------- PATHS ----------
    private PathChain score1;
    private PathChain line1PrePath;
    private PathChain intakeLine1Path;
    private PathChain score2;

    private PathChain line2PrePath;
    private PathChain intakeLine2Path;
    private PathChain line2RetreatPath;
    private PathChain score3;

    private PathChain line3PrePath;
    private PathChain intakeLine3Path;
    private PathChain score4;

    private PathChain leverPrePath;
    private PathChain leverPath;
    private PathChain leverScorePath;

    @Override
    public void init() {
        opModeTimer = new Timer();
        waitTimer = new Timer();
        poseTimer = new ElapsedTime();

        opModeTimer.resetTimer();
        waitTimer.resetTimer();
        poseTimer.reset();

        follower = Constants.createFollower(hardwareMap);
        buildPaths();
        follower.setStartingPose(startPose);

        shooter = new shooter();
        shooter.init(hardwareMap);
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
        telemetry.addData("At Parametric End", follower.atParametricEnd());
        telemetry.addData("Velocity", follower.getVelocity());
        telemetry.addData("Shots", shooter.ShotsRemaining);
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
                ie.setVelocity(ieP);
                follower.setMaxPower(1.0);
                startPath(line1PrePath, true);
                pathState = 5;
                break;

            // Wait until at Line 1 pre-intake
            case 5:
                ie.setVelocity(ieP);
                if (poseDone()) {
                    pathState = 6;
                }
                break;

            // Drive through Line 1 intake
            case 6:
                ie.setVelocity(ieP);
                follower.setMaxPower(0.75);
                startPath(intakeLine1Path, true);
                pathState = 7;
                break;

            // Wait until Line 1 intake finishes
            case 7:
                ie.setVelocity(ieP);
                if (poseDone()) {
                    follower.setMaxPower(1.0);
                    pathState = 8;
                }
                break;

            // Return to score second volley
            case 8:
                ie.setVelocity(ieP);
                follower.setMaxPower(1.0);
                startPath(score2, true);
                pathState = 9;
                break;

            // Wait until back at score
            case 9:
                ie.setVelocity(ieP);
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
                ie.setVelocity(ieP);
                follower.setMaxPower(1.0);
                startPath(line2PrePath, true);
                pathState = 13;
                break;

            // Wait until at Line 2 pre-intake
            case 13:
                ie.setVelocity(ieP);
                if (poseDone()) {
                    pathState = 14;
                }
                break;

            // Drive through Line 2 intake
            case 14:
                ie.setVelocity(ieP);
                follower.setMaxPower(0.75);
                startPath(intakeLine2Path, true);
                pathState = 15;
                break;

            // Wait until Line 2 intake finishes
            case 15:
                ie.setVelocity(ieP);
                if (poseDone()) {
                    follower.setMaxPower(1.0);
                    pathState = 16;
                }
                break;

            // Retreat from Line 2 back to pre-pose
            case 16:
                ie.setVelocity(ieP);
                follower.setMaxPower(1.0);
                startPath(line2RetreatPath, true);
                pathState = 17;
                break;

            // Wait until retreat finishes
            case 17:
                ie.setVelocity(ieP);
                if (poseDone()) {
                    pathState = 18;
                }
                break;

            // Return to score third volley
            case 18:
                ie.setVelocity(ieP);
                follower.setMaxPower(1.0);
                startPath(score3, true);
                pathState = 19;
                break;

            // Wait until back at score
            case 19:
                ie.setVelocity(ieP);
                if (poseDone()) {
                    pathState = 20;
                }
                break;

            // Shoot third volley
            case 20:
                shooter.shoot();
                pathState = 21;
                break;

            // Wait until third volley finishes
            case 21:
                if (shooter.ShotsRemaining <= 0) {
                    pathState = 22;
                }
                break;

            // Drive to Line 3 pre-intake
            case 22:
                ie.setVelocity(ieP);
                follower.setMaxPower(1.0);
                startPath(line3PrePath, true);
                pathState = 23;
                break;

            // Wait until at Line 3 pre-intake
            case 23:
                ie.setVelocity(ieP);
                if (poseDone()) {
                    pathState = 24;
                }
                break;

            // Drive through Line 3 intake
            case 24:
                ie.setVelocity(ieP);
                follower.setMaxPower(0.75);
                startPath(intakeLine3Path, true);
                pathState = 25;
                break;

            // Wait until Line 3 intake finishes
            case 25:
                ie.setVelocity(ieP);
                if (poseDone()) {
                    follower.setMaxPower(1.0);
                    pathState = 26;
                }
                break;

            // Return to score fourth volley
            case 26:
                ie.setVelocity(ieP);
                follower.setMaxPower(1.0);
                startPath(score4, true);
                pathState = 27;
                break;

            // Wait until back at score
            case 27:
                ie.setVelocity(ieP);
                if (poseDone()) {
                    pathState = 28;
                }
                break;

            // Shoot fourth volley
            case 28:
                shooter.shoot();
                pathState = 29;
                break;

            // Wait until fourth volley finishes
            case 29:
                if (shooter.ShotsRemaining <= 0) {
                    pathState = 30;
                }
                break;

            // Done
            case 30:
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

        line2RetreatPath = follower.pathBuilder()
                .addPath(new BezierLine(intakeLine2Pose, line2PrePose))
                .setLinearHeadingInterpolation(intakeLine2Pose.getHeading(), line2PrePose.getHeading())
                .build();

        score3 = follower.pathBuilder()
                .addPath(new BezierLine(line2PrePose, scorePose))
                .setLinearHeadingInterpolation(line2PrePose.getHeading(), scorePose.getHeading())
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