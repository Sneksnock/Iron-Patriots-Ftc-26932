package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.teamcode.shooter.ShootingState.IDLE;
import static org.firstinspires.ftc.teamcode.shooter.ShootingState.IDLE2;
import static org.firstinspires.ftc.teamcode.shooter.ie;
import static org.firstinspires.ftc.teamcode.shooter.ieP;
import static org.firstinspires.ftc.teamcode.shooter.off;
import static org.firstinspires.ftc.teamcode.shooter.offW;
import static org.firstinspires.ftc.teamcode.shooter.shootingState;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@Autonomous(name = "Blue(Pedro)", group = "Examples")
public class Blue_Side_Pedro extends OpMode {

    private shooter shooter;
    private Follower follower;

    public Timer pathTimer, opModeTimer, waitTimer;


    private int pathState = 0;
    private int wait = 1500 ;
    private boolean shotRequested = false;

    private  Pose startPose = new Pose(33.916, 126.968, (2.4196));
    private Pose scorePose = new Pose(72, 85.281, (2.3));
    private  Pose line1Pre =  new Pose(48.376,86, 3.070);
    private Pose intake2Pose = new Pose(40, 86, 3.070);
    private Pose intake3OutsidePose = new Pose(13, 79,3.070);
    private Pose tempPose = new Pose(25, 60, 3.070);
    private Pose leverPose = new Pose(21.0, 64.082, 2.566);


    private PathChain score1, l1Pos, intakeL12, intakeL13, score2, lever, score, lever2;

    @Override
    public void init() {
        shooter = new shooter();
        shooter.init(hardwareMap);
        pathTimer = new Timer();
        opModeTimer = new Timer();
        opModeTimer.resetTimer();
        waitTimer = new Timer();
        waitTimer.resetTimer();
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
        //telemetry.addData("At Parametric End", follower.atParametricEnd());
        telemetry.addData("Velocity", follower.getVelocity());
        telemetry.update();
    }

    public void autonomousPathUpdate() {
        switch (pathState) {

            case 0:
                ie.setPower(ieP);
                follower.followPath(score1, true);
                if (!Schmovin()) {
                    pathTimer.resetTimer();
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
                follower.followPath(intakeL12, true);
                if (!Schmovin()){
                    pathState = 3;
                    }

                break;

           case 3:
               ie.setPower(ieP);
                follower.followPath(intakeL13, true);
                if (!Schmovin()) {
                    pathState = 4;
                    break;
                }
            case 4:
                follower.followPath(score2, true);
                if (!Schmovin()){
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
                follower.followPath(lever2, true);
                if (!Schmovin()){
                    waitTimer.resetTimer();
                    pathState = 7;
                }
                break;
            case 7:
                ie.setPower(ieP);
                follower.followPath(lever, true);
                if (!Schmovin()){
                    waitTimer.resetTimer();
                pathState = 8;
                }
                break;
            case 8:
                if (waitTimer.getElapsedTime() >= wait){
                    pathState = 9;
                    ie.setPower(off);
                }
            break;
            case 9:
             follower.followPath(score, true);
             if (!Schmovin()){
                 shooter.shoot();
                 pathState = 10;
             }
             break;
            case 10:
                if(shooter.ShotsRemaining <= 0){
                    pathState = 11;
                }
                break;
            case 11:
                ie.setPower(ieP);
                follower.followPath(lever, true);
                if (!Schmovin()){
                    waitTimer.resetTimer();
                    pathState = 12;
                }
                break;
            case 12:
                if (waitTimer.getElapsedTime() >= wait){
                    pathState = 13;
                }
                break;
            case 13:

                follower.followPath(score, true);
                if (!Schmovin()){
                    shooter.shoot();
                    pathState = 14;
                }
                break;
            case 14:
                if(shooter.ShotsRemaining <= 0){
                    pathState = 15;
                }
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
        lever2 = follower.pathBuilder()
                .addPath(new BezierLine(scorePose, tempPose))
                .setLinearHeadingInterpolation(scorePose.getHeading(), leverPose.getHeading())
                .build();
        lever = follower.pathBuilder()
                .addPath(new BezierLine(tempPose, leverPose))
                .setLinearHeadingInterpolation(tempPose.getHeading(), leverPose.getHeading())
                .build();
        score = follower.pathBuilder()
                .addPath(new BezierLine(leverPose, scorePose))
                .setLinearHeadingInterpolation(leverPose.getHeading(), scorePose.getHeading())
                .build();
    }
}
