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

@Autonomous(name = "Blue Far Side", group = "Examples")
public class Far_Side_Pedro extends OpMode {
    private shooter shooter;
    private Follower follower;

    public Timer opModeTimer, waitTimer;
    private ElapsedTime poseTimer;

    private int pathState = 0;
    private static final double POSE_TIMEOUT = 4.0;

    /// ---------- POSE DATA ----------
    private Pose startPose = new Pose(47.2, 7.3, 1.6);
    private Pose scorePose = new Pose(69, 15, 2.05);
    private Pose line1PrePose = new Pose(47.5, 34.9, 3.07);
    private Pose intake2Pose = new Pose(27, 57.5, 3.07);
    private Pose intake3OutsidePose = new Pose(20.5, 45.4, 3.07);
    private Pose leverPrePose = new Pose(25, 60, 2.56);
    private Pose leverPose = new Pose(19.0, 62.082, 2.56);
    private Pose line2PrePose = new Pose(66, 35, 3.07);
    private Pose line2Pose = new Pose(23, 57, 3.07);

    private PathChain score1, l1Pos, intakeL12, intakeL13, score2, lever, score, leverPre, L2Pre, L2, L2score;

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

        /// ----------- TELEMETRY ----------
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
                shooter.shootFar();
                pathState = 3;
                break;

            // Wait until first volley finishes
            case 3:
                if (shooter.ShotsRemaining <= 0) {
                    pathState = 4;
                }
                break;

            // Drive to first pre-intake
            case 4:
                ie.setPower(ieP);
                follower.setMaxPower(1.0);
                startPath(l1Pos, true);
                pathState = 5;
                break;

            // Wait until at first pre-intake
            case 5:
                ie.setPower(ieP);
                if (poseDone()) {
                    pathState = 6;
                }
                break;

            // Drive through first intake
            case 6:
                ie.setPower(ieP);
                follower.setMaxPower(0.75);
                startPath(intakeL12, true);
                pathState = 7;
                break;

            // Wait until first intake finishes
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
                shooter.shootFar();
                pathState = 11;
                break;

            // Wait until second volley finishes
            case 11:
                if (shooter.ShotsRemaining <= 0) {
                    pathState = 12;
                }
                break;

            // Drive to second pre-intake
            case 12:
                ie.setPower(ieP);
                follower.setMaxPower(1.0);
                startPath(L2Pre, true);
                pathState = 13;
                break;

            // Wait until at second pre-intake
            case 13:
                ie.setPower(ieP);
                if (poseDone()) {
                    pathState = 14;
                }
                break;

            // Drive through second intake
            case 14:
                ie.setPower(ieP);
                follower.setMaxPower(0.75);
                startPath(L2, true);
                pathState = 15;
                break;

            // Wait until second intake finishes
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
                startPath(L2score, true);
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
                shooter.shootFar();
                pathState = 19;
                break;

            // Wait until third volley finishes
            case 19:
                if (shooter.ShotsRemaining <= 0) {
                    pathState = 20;
                }
                break;

            // Done
            case 20:
                ie.setPower(0);
                follower.setMaxPower(1.0);
                break;
        }
    }

    public void buildPaths() {
        score1 = follower.pathBuilder()
                .addPath(new BezierLine(startPose, scorePose))
                .setLinearHeadingInterpolation(startPose.getHeading(), scorePose.getHeading())
                .build();

        l1Pos = follower.pathBuilder()
                .addPath(new BezierLine(scorePose, line1PrePose))
                .setLinearHeadingInterpolation(scorePose.getHeading(), line1PrePose.getHeading())
                .build();

        intakeL12 = follower.pathBuilder()
                .addPath(new BezierLine(line1PrePose, intake3OutsidePose))
                .setLinearHeadingInterpolation(line1PrePose.getHeading(), intake3OutsidePose.getHeading())
                .build();

        score2 = follower.pathBuilder()
                .addPath(new BezierLine(intake3OutsidePose, scorePose))
                .setLinearHeadingInterpolation(intake3OutsidePose.getHeading(), scorePose.getHeading())
                .build();

        L2Pre = follower.pathBuilder()
                .addPath(new BezierLine(scorePose, line2PrePose))
                .setLinearHeadingInterpolation(scorePose.getHeading(), line2PrePose.getHeading())
                .build();

        L2 = follower.pathBuilder()
                .addPath(new BezierLine(line2PrePose, line2Pose))
                .setLinearHeadingInterpolation(line2PrePose.getHeading(), line2Pose.getHeading())
                .build();

        L2score = follower.pathBuilder()
                .addPath(new BezierLine(line2Pose, scorePose))
                .setLinearHeadingInterpolation(line2Pose.getHeading(), scorePose.getHeading())
                .build();

        leverPre = follower.pathBuilder()
                .addPath(new BezierLine(scorePose, leverPrePose))
                .setLinearHeadingInterpolation(scorePose.getHeading(), leverPrePose.getHeading())
                .build();

        lever = follower.pathBuilder()
                .addPath(new BezierLine(leverPrePose, leverPose))
                .setLinearHeadingInterpolation(leverPrePose.getHeading(), leverPose.getHeading())
                .build();

        score = follower.pathBuilder()
                .addPath(new BezierLine(leverPose, scorePose))
                .setLinearHeadingInterpolation(leverPose.getHeading(), scorePose.getHeading())
                .build();
    }
}