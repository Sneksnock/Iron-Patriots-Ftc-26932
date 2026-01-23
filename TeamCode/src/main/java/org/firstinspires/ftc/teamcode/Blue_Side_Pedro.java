package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.teamcode.shooter.follower;
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

@Autonomous(name = "Blue(Pedro)", group = "Examples")
public class Blue_Side_Pedro extends OpMode {

    private shooter shooter;

    private Timer pathTimer, opModeTimer;

    public static int pathState = 0;
    private boolean shotRequested = false;

    public final Pose startPose = new Pose(33.916, 126.968, (2.4196));
    public final Pose scorePose = new Pose(71, 80.281, (2.3));
    public final Pose line1Pre =  new Pose(47.376,81.564, 3.070);
    public final Pose intake2Pose = new Pose(42, 81, (3.070));
    public final Pose intake3OutsidePose = new Pose(26, 86, (3.070));
    public final Pose leverPose = new Pose(22.583, 60.082, (2.566));

    private PathChain score1, l1Pos, intakeL12, intakeL13, score2, lever;

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
        telemetry.update();
    }

    public void autonomousPathUpdate() {
        switch (pathState) {
            case 0:
                follower.followPath(score1, true);
                shoot();

                break;

            case 1:
                ie.setPower(ieP);
                follower.followPath(l1Pos, true);
                if (!Schmovin()) pathState = 2;
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
                   shoot();


                break;

            case 5:
                ie.setPower(.75);
                follower.followPath(lever, true);
                break;
        }
    }

    public boolean Schmovin() {
        return Math.abs(follower.getHeadingError()) > .98 ||
                !follower.atParametricEnd() ||
                follower.getVelocity().getMagnitude() > 0.90 ||
                Math.abs(follower.getAngularVelocity()) > .90;
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

        lever = follower.pathBuilder()
                .addPath(new BezierLine(scorePose, leverPose))
                .setLinearHeadingInterpolation(scorePose.getHeading(), leverPose.getHeading())
                .build();
    }
}
