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

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.shooter;

@Autonomous(name = "RED Far Side", group = "Examples")
public class Far_Side_Pedro extends OpMode {
    private shooter shooter;
    private Follower follower;

    public Timer pathTimer, opModeTimer, waitTimer;


    private int pathState = 0;
    private int wait = 1500 ;
    private boolean shotRequested = false;



    private  Pose startPose = new Pose(47.2, 7.3, (1.6));
    private Pose scorePose = new Pose(69, 15, (2.05));
    private  Pose line1PrePose =  new Pose(47.5,34.9, 3.07);
    private Pose intake2Pose = new Pose(27, 57.5, 3.07);
    private Pose intake3OutsidePose = new Pose(20.5, 45.4,3.07);
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
        //telemetry.addData("At Parametric End", follower.atParametricEnd());
        telemetry.addData("Velocity", follower.getVelocity());
        telemetry.update();
    }

    public void autonomousPathUpdate() {
        switch (pathState) {

            case 0:
                follower.followPath(score1, true);
                if (!Schmovin()) {
                    pathTimer.resetTimer();
                    shooter.shootFar();
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
                if (!Schmovin()){
                    pathState = 3;
                }
                break;
            case 3:
                ie.setPower(ieP);
                follower.followPath(intakeL12, true);
                if (!Schmovin()){
                    pathState = 4;
                }
                break;
            case 4:
                follower.followPath(score2, true);
                if (!Schmovin()){
                    shooter.shootFar();
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
                follower.followPath(L2Pre, true);
                if (!Schmovin()){
                    waitTimer.resetTimer();
                    pathState = 7;
                }
                break;
          /*  case 7:
                ie.setPower(ieP);
                follower.followPath(L2, true);
                if (!Schmovin()){
                    waitTimer.resetTimer();
                    pathState = 8;
                }
                break;
            case 8:
                ie.setPower(ieP);
                follower.followPath(L2score, true);
                if (!Schmovin()){
                    shooter.shoot();
                    waitTimer.resetTimer();
                    pathState = 9;
                }
                break;
            case 9:

                if (shooter.ShotsRemaining <= 0) {
                    pathState = 10;
                }
                break;
           case 9:
                ie.setPower(ieP);
                follower.followPath(leverPre, true);
                if (!Schmovin()){
                    waitTimer.resetTimer();
                    pathState = 10;
                }
                break;
              case 10:
                follower.followPath(lever, true);
                if (!Schmovin()){
                    waitTimer.resetTimer();
                pathState = 11;
                }
                break;
            case 11:
                if (waitTimer.getElapsedTime() >= wait){
                    pathState = 12;
                }
            break;
            case 12:
             follower.followPath(score, true);
             if (!Schmovin()){
                 shooter.shoot();
                 pathState = 13;
             }
             break;
            case 13:
                if(shooter.ShotsRemaining <= 0){
                    pathState = 14;
                }
                break;
            case 14:ie.setPower(ieP);
                follower.followPath(lever, true);
                if (!Schmovin()){
                    waitTimer.resetTimer();
                    pathState = 15;
                }
                break;
            case 15:ie.setPower(ieP);
                if (waitTimer.getElapsedTime() >= wait){
                    pathState = 16;
                    ie.setPower(ieP);
                }
                break;
            case 16:
                ie.setPower(ieP);
                follower.followPath(score, true);
                if (!Schmovin()){
                    shooter.shoot();
                    pathState = 17;
                }
                break;
            case 17:
                if(shooter.ShotsRemaining <= 0){
                    pathState = 18;
                }
                break;*/


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
                .addPath(new BezierLine(scorePose, line1PrePose))
                .setLinearHeadingInterpolation(scorePose.getHeading(), line1PrePose.getHeading())
                .build();

        intakeL12 = follower.pathBuilder()
                .addPath(new BezierLine(line1PrePose, intake3OutsidePose))
                .setLinearHeadingInterpolation(line1PrePose.getHeading(), intake3OutsidePose.getHeading())
                .build();

        //intakeL13 = follower.pathBuilder()
        //  .addPath(new BezierLine(intake2Pose, intake3OutsidePose))
        //   .setLinearHeadingInterpolation(intake2Pose.getHeading(), intake3OutsidePose.getHeading())
        // .build();

        score2 = follower.pathBuilder()
                .addPath(new BezierLine(intake3OutsidePose, scorePose))
                .setLinearHeadingInterpolation(intake3OutsidePose.getHeading(), scorePose.getHeading())
                .build();
        L2Pre = follower.pathBuilder()
                .addPath(new BezierLine(scorePose, line2PrePose))
                .setLinearHeadingInterpolation(scorePose.getHeading(), line2PrePose.getHeading())
                .build();
        L2 = follower.pathBuilder()
                .addPath(new BezierLine( line2PrePose, line2Pose))
                .setLinearHeadingInterpolation(line2PrePose.getHeading(), line2Pose.getHeading())
                .build();
        L2score = follower.pathBuilder()
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
