package org.firstinspires.ftc.teamcode.pedroPathing;

import static org.firstinspires.ftc.teamcode.shooter.ie;
import static org.firstinspires.ftc.teamcode.shooter.ieP;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.shooter;

@Autonomous(name = "Blue Far Side", group = "Examples")
public class Far_Side_Pedro extends OpMode {

    private shooter shooter;
    private Follower follower;

    public Timer pathTimer, opModeTimer, waitTimer;

    private int pathState = 0;
    private int wait = 1500;

    private Pose startPose = new Pose(47.2, 7.3, (1.6));
    private Pose scorePose = new Pose(69, 15, (2.05));
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

        pathTimer = new Timer();
        opModeTimer = new Timer();
        waitTimer = new Timer();

        pathTimer.resetTimer();
        opModeTimer.resetTimer();
        waitTimer.resetTimer();

        shooter.init(hardwareMap);

        follower = Constants.createFollower(hardwareMap);

        buildPaths();

        follower.setStartingPose(startPose);
    }

    @Override
    public void loop() {

        follower.update();
        shooter.update();

        autonomousPathUpdate();

        telemetry.addData("Path State", pathState);
        telemetry.addData("Shooter State", shooter.getState());
        telemetry.addData("Follower Busy", follower.isBusy());
        telemetry.addData("Velocity", follower.getVelocity());
        telemetry.addData("Shots Remaining", shooter.ShotsRemaining);
        telemetry.update();
    }

    public void autonomousPathUpdate() {

        switch (pathState) {

            case 0:
                follower.followPath(score1, true);
                pathState = 1;
                break;

            case 1:

                if (!follower.isBusy()) {

                    shooter.shootFar();

                    pathState = 2;
                }

                break;

            case 2:

                if (shooter.ShotsRemaining <= 0) {

                    ie.setPower(ieP);

                //    follower.followPath(l1Pos, true);

                    pathState = 3;
                }

            /*    break;

            case 3:

                ie.setPower(ieP);

                if (!follower.isBusy()) {

                    follower.followPath(intakeL12, true);

                    pathState = 4;
                }

                break;

            case 4:

                ie.setPower(ieP);

                if (!follower.isBusy()) {

                    follower.followPath(score2, true);

                    pathState = 5;
                }

                break;

            case 5:

                if (!follower.isBusy()) {

                    shooter.shootFar();

                    pathState = 6;
                }

                break;

            case 6:

                if (shooter.ShotsRemaining <= 0) {

                    ie.setPower(ieP);

                    follower.followPath(L2Pre, true);

                    pathState = 7;
                }

                break;

            case 7:

                ie.setPower(ieP);

                if (!follower.isBusy()) {

                    follower.followPath(L2, true);

                    pathState = 8;
                }

                break;

            case 8:

                ie.setPower(ieP);

                if (!follower.isBusy()) {

                    follower.followPath(L2score, true);

                    pathState = 9;
                }

                break;

            case 9:

                ie.setPower(ieP);

                if (!follower.isBusy()) {

                    shooter.shootFar();

                    pathState = 10;
                }

                break;

            case 10:

                if (shooter.ShotsRemaining <= 0) {

                    pathState = 11;
                }

                break; */

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