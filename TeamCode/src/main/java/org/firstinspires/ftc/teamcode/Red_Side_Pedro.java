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

@Autonomous(name = "Red_Side_Pedro", group = "Examples")
public class Red_Side_Pedro extends OpMode {

    private Follower follower;
    private shooter shooter;
    private Timer pathTimer, opModeTimer, waitTimer;
    private int wait = 2000;
    private int pathState = 0;
    private boolean shotRequested = false;

    private final Pose startPose = new Pose(124.311, 123.794, 0.739);
    private final Pose scorePose = new Pose(80, 87.281, 0.712);
    private final Pose line1PrePose = new Pose(102.834, 82.072, 0.0);
    private final Pose intake2Pose = new Pose(125.172, 81.622, 0.02);
    private final Pose line2PrePose = new Pose( 102.56, 57.89, 0.06);
    private final Pose line2Pose = new Pose (127.34, 53.68, 0.01);
    private final Pose leverPrePose = new Pose (139, 60, 0.03);
    private final Pose leverPose = new Pose (108, 66, 2.56);

    private PathChain score1, l1Pos, intakeL12, L2Pre, L2, L2Score, leverPre, lever, score, score2;

    @Override
    public void init() {
        pathTimer = new Timer();
        opModeTimer = new Timer();
        waitTimer = new Timer();
        opModeTimer.resetTimer();

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

        telemetry.addData("Path State", pathState);
        telemetry.addData("Shooter State", shooter.getState());
        telemetry.addData("At Parametric End", follower.atParametricEnd());
        telemetry.addData("Velocity", follower.getVelocity());
        telemetry.addData("shots",shooter.ShotsRemaining);
        telemetry.update();
    }




    public boolean Schmovin() {
        return Math.abs(follower.getHeadingError()) > .98 ||
                !follower.atParametricEnd() ||
                follower.getVelocity().getMagnitude() > 0.90 ||
                Math.abs(follower.getAngularVelocity()) > .90;
    }


    public void autonomousPathUpdate() {
        switch (pathState) {
            case 0:
                follower.followPath(score1, true);
                if (!Schmovin()) {
                    shooter.shoot();
                    pathState = 1;
                }
                break;

            case 1:

                if (shooter.ShotsRemaining <= 0) {
                    pathState = 2;
                }
                break;
            case 2:
                ie.setPower(ieP);
                follower.followPath(l1Pos, true);
                if (!Schmovin()) {
                    pathState = 3;
                }
                break;
            case 3:
                ie.setPower(ieP);
                follower.followPath(intakeL12, true);
                if (!Schmovin()) {
                    pathState = 4;
                }
                break;
            case 4:
                follower.followPath(score2, true);
                if (!Schmovin()) {
                    shooter.shoot();
                    pathState = 5;
                }
                break;
            case 5:

                if (shooter.ShotsRemaining <= 0) {
                    pathState = 6;
                }
                break;
            case 6:
                ie.setPower(ieP);
                follower.followPath(L2Pre);
                if (!Schmovin()) {
                    pathState = 7;
                }
                break;
            case 7:
                ie.setPower(ieP);
                follower.followPath(L2);
                if (!Schmovin()) {
                    pathState = 8;
                    waitTimer.resetTimer();
                }
                break;
            case 8:
                follower.followPath(L2Score);
                if (!Schmovin()) {
                    pathState = 9;
                    waitTimer.resetTimer();
                }
                break;
            case 9:
                follower.followPath(score);
                if(!Schmovin()) {
                    shooter.shoot();
                    pathState = 10;
                }
                break;
            case 10:
                if (shooter.ShotsRemaining <= 0) {
                    pathState = 11;
                }
                break;
        }
    }

    public void buildPaths() {
        score1 = follower.pathBuilder()
                .addPath(new BezierLine(startPose, scorePose))
                .setLinearHeadingInterpolation(startPose.getHeading(), scorePose.getHeading())
                .build();

        l1Pos = follower.pathBuilder()
                .addPath(new BezierLine(scorePose, line1PrePose ))
                .setLinearHeadingInterpolation(scorePose.getHeading(), line1PrePose.getHeading())
                .build();

        intakeL12 = follower.pathBuilder()
                .addPath(new BezierLine(line1PrePose, intake2Pose))
                .setLinearHeadingInterpolation(line1PrePose.getHeading(), intake2Pose.getHeading())
                .build();
        score2 = follower.pathBuilder()
                .addPath(new BezierLine(intake2Pose, scorePose))
                .setLinearHeadingInterpolation(line2Pose.getHeading(), scorePose.getHeading())
                .build();
        L2Pre = follower.pathBuilder()
                .addPath(new BezierLine(scorePose, line2PrePose))
                .setLinearHeadingInterpolation(scorePose.getHeading(), line1PrePose.getHeading())
                .build();
        L2 = follower.pathBuilder()
                .addPath(new BezierLine( line2PrePose, line2Pose))
                .setLinearHeadingInterpolation(line2PrePose.getHeading(), line2Pose.getHeading())
                .build();
        L2Score = follower.pathBuilder()
                .addPath(new BezierLine( line2Pose, scorePose))
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
