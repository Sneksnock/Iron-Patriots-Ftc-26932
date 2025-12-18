package org.firstinspires.ftc.teamcode;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.sun.tools.javac.code.Attribute;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

import java.net.ContentHandler;

@Autonomous (name = "Pedro Test", group = "Examples")
public class Test_Pedro extends OpMode {
    private Follower follower;
    private Timer pathTimer, actionTimer, opModeTimer;

    private int pathState;
    private final Pose startPose = new Pose(0 ,0, Math.toRadians(0));
    private final Pose scorePose = new Pose(0, .5, Math.toRadians(0));
    private PathChain grabPickup1;

    public void autonomousPathUpdate() {
        switch (pathState) {
            case 0:
                follower.followPath(grabPickup1);
        }
    }

    @Override
    public void loop() {
        follower.update();
        autonomousPathUpdate();

        //tele
        telemetry.addData("Path State", pathState);
        telemetry.addData("x", follower.getPose().getX());
        telemetry.addData("y", follower.getPose().getX());
        telemetry.addData("heading", follower.getPose().getHeading());
        telemetry.update();

    }
    @Override
    public void init() {
        pathTimer =  new Timer();
        opModeTimer = new Timer();
        opModeTimer.resetTimer();


        follower = Constants.createFollower(hardwareMap);
        buildPaths();
        follower.setStartingPose(startPose);
    }
    public void buildPaths() {
        grabPickup1 = follower.pathBuilder()
                .addPath(new BezierLine(startPose, scorePose))
                .setLinearHeadingInterpolation(startPose.getHeading(), scorePose.getHeading())
                .build();

    }
}
