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

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@Autonomous(name = "Red Side", group = "Examples")
public class Red_Side_Pedro extends OpMode {

    private Follower follower;
    private shooter shooter;
    private Timer opModeTimer, waitTimer;

    private int pathState = 0;
/// ---------- POSE DATA ----------
    private final Pose startPose = new Pose(124.311, 123.794, 0.739);
    private final Pose scorePose = new Pose(72, 129.5, 0.17);
    private final Pose line1PrePose = new Pose(102.5, 86.75, 0.0);
    private final Pose intakeline1Pose = new Pose(128.0, 81.0, 0.02);
    private final Pose line2PrePose = new Pose(102.5, 61.5, 0.06);

    // private final Pose intake3PrePose = new Pose(102.5, 39.5, 0.02);
    private final Pose intakeline2Pose = new Pose(138, 56.0, 0.01);
    private final Pose leverPrePose = new Pose(139, 60, 0.03);
    private final Pose leverPose = new Pose(108, 66, 2.56);

    private PathChain score1, l1Pre, intakeL1, score2, L2Pre, intakeL2, score3, leverPre, lever, leverscore;

    @Override
    public void init() {
        opModeTimer = new Timer();
        waitTimer = new Timer();
        opModeTimer.resetTimer();
        waitTimer.resetTimer();

        follower = Constants.createFollower(hardwareMap);
        buildPaths();
        follower.setStartingPose(startPose);

        shooter = new shooter();
        shooter.init(hardwareMap);
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
        telemetry.addData("At Parametric End", follower.atParametricEnd());
        telemetry.addData("Velocity", follower.getVelocity());
        telemetry.addData("Shots", shooter.ShotsRemaining);
        telemetry.update();
    }

    /// ---------- PATHING ----------
    public void autonomousPathUpdate() {
        switch (pathState) {

            // Drive to first score
            case 0:
                follower.followPath(score1, true);
                pathState = 1;
                break;

            // Wait until at first score
            case 1:
                if (!follower.isBusy()) {
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

            // Drive to first pre-intake
            case 4:
                ie.setPower(ieP);
                follower.followPath(l1Pre, true);
                pathState = 5;
                break;

            // Wait until at first pre-intake
            case 5:
                ie.setPower(ieP);
                if (!follower.isBusy()) {
                    pathState = 6;
                }
                break;

            // Drive through first intake
            case 6:
                ie.setPower(ieP);
                follower.followPath(intakeL1, true);
                pathState = 7;
                break;

            // Wait until first intake finishes
            case 7:
                ie.setPower(ieP);
                if (!follower.isBusy()) {
                    pathState = 8;
                }
                break;

            // Return to score second volley
            case 8:
                ie.setPower(ieP);
                follower.followPath(score2, true);
                pathState = 9;
                break;

            // Wait until back at score
            case 9:
                ie.setPower(ieP);
                if (!follower.isBusy()) {
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

            // Drive to second pre-intake
            case 12:
                ie.setPower(ieP);
                follower.followPath(L2Pre, true);
                pathState = 13;
                break;

            // Wait until at second pre-intake
            case 13:
                ie.setPower(ieP);
                if (!follower.isBusy()) {
                    pathState = 14;
                }
                break;

            // Drive through second intake
            case 14:
                ie.setPower(ieP);
                follower.followPath(intakeL2, true);
                pathState = 15;
                break;

            // Wait until second intake finishes
            case 15:
                ie.setPower(ieP);
                if (!follower.isBusy()) {
                    pathState = 16;
                }
                break;

            // Return to score third volley
            case 16:
                ie.setPower(ieP);
                follower.followPath(score3, true);
                pathState = 17;
                break;

            // Wait until back at score
            case 17:
                ie.setPower(ieP);
                if (!follower.isBusy()) {
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

            // Done
            case 20:
                ie.setPower(0);
                break;
        }
    }

    public void buildPaths() {
        score1 = follower.pathBuilder()
                .addPath(new BezierLine(startPose, scorePose))
                .setLinearHeadingInterpolation(startPose.getHeading(), scorePose.getHeading())
                .build();

        l1Pre = follower.pathBuilder()
                .addPath(new BezierLine(scorePose, line1PrePose))
                .setLinearHeadingInterpolation(scorePose.getHeading(), line1PrePose.getHeading())
                .build();

        intakeL1 = follower.pathBuilder()
                .addPath(new BezierLine(line1PrePose, intakeline1Pose))
                .setLinearHeadingInterpolation(line1PrePose.getHeading(), intakeline1Pose.getHeading())
                .build();

        score2 = follower.pathBuilder()
                .addPath(new BezierLine(intakeline1Pose, scorePose))
                .setLinearHeadingInterpolation(intakeline1Pose.getHeading(), scorePose.getHeading())
                .build();

        L2Pre = follower.pathBuilder()
                .addPath(new BezierLine(scorePose, line2PrePose))
                .setLinearHeadingInterpolation(scorePose.getHeading(), line2PrePose.getHeading())
                .build();

        intakeL2 = follower.pathBuilder()
                .addPath(new BezierLine(line2PrePose, intakeline2Pose))
                .setLinearHeadingInterpolation(line2PrePose.getHeading(), intakeline2Pose.getHeading())
                .build();

        score3 = follower.pathBuilder()
                .addPath(new BezierLine(intakeline2Pose, scorePose))
                .setLinearHeadingInterpolation(intakeline2Pose.getHeading(), scorePose.getHeading())
                .build();

        leverPre = follower.pathBuilder()
                .addPath(new BezierLine(scorePose, leverPrePose))
                .setLinearHeadingInterpolation(scorePose.getHeading(), leverPrePose.getHeading())
                .build();

        lever = follower.pathBuilder()
                .addPath(new BezierLine(leverPrePose, leverPose))
                .setLinearHeadingInterpolation(leverPrePose.getHeading(), leverPose.getHeading())
                .build();

        leverscore = follower.pathBuilder()
                .addPath(new BezierLine(leverPose, scorePose))
                .setLinearHeadingInterpolation(leverPose.getHeading(), scorePose.getHeading())
                .build();
    }
}