package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.teamcode.shooter.ShotsRemaining;
import static org.firstinspires.ftc.teamcode.shooter.ie;
import static org.firstinspires.ftc.teamcode.shooter.ieP;
import static org.firstinspires.ftc.teamcode.shooter.shoot;

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
    private Timer pathTimer, opModeTimer;
    private int wait = 2000;
    private int pathState = 0;
    private boolean shotRequested = false;

    private final Pose startPose = new Pose(124.311, 121.794, 0.739);
    private final Pose scorePose = new Pose(80, 87.281, 0.712);
    private final Pose line1Pre = new Pose(106.834, 82.316, 0.08);
    private final Pose intake2Pose = new Pose(111.975, 80.288, 0.02);
    private final Pose intake3OutsidePose = new Pose(127.339, 80.754, 0.03);

    private PathChain score1, l1Pos, intakeL12, intakeL13, score2;

    @Override
    public void init() {
        pathTimer = new Timer();
        opModeTimer = new Timer();
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
        telemetry.addData("shots",ShotsRemaining);
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

                if (ShotsRemaining <= 0) {
                    pathState = 2;
                }
                break;
            case 2:
                ie.setPower(ieP);
                follower.followPath(intakeL12, true);
                if (!Schmovin()) pathState = 3;
                break;

            case 3:
                ie.setPower(ieP);
                follower.followPath(intakeL13, true);
                if (!Schmovin()) pathState = 4;
                break;

            case 4:
                follower.followPath(score2, true);
                if (!Schmovin()) {
                    shoot();
                    pathState = 5;
                }
                break;
            case 5:

                if (ShotsRemaining <= 0) {
                    pathState = 6;
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
                .addPath(new BezierLine(scorePose, line1Pre))
                .setLinearHeadingInterpolation(scorePose.getHeading(), line1Pre.getHeading())
                .build();

        intakeL12 = follower.pathBuilder()
                .addPath(new BezierLine(line1Pre, intake2Pose))
                .setLinearHeadingInterpolation(line1Pre.getHeading(), intake2Pose.getHeading())
                .build();

        intakeL13 = follower.pathBuilder()
                .addPath(new BezierLine(intake2Pose, intake3OutsidePose))
                .setLinearHeadingInterpolation(intake2Pose.getHeading(), intake3OutsidePose.getHeading())
                .build();

        score2 = follower.pathBuilder()
                .addPath(new BezierLine(intake3OutsidePose, scorePose))
                .setLinearHeadingInterpolation(intake3OutsidePose.getHeading(), scorePose.getHeading())
                .build();
    }
}
