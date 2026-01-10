package org.firstinspires.ftc.teamcode;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@Autonomous(name = "Blue(Pedro)", group = "Examples")
public class Blue_Side_Pedro extends OpMode {

    // ---------------- HARDWARE ----------------
    private Follower follower;
    DcMotorEx Lf, Rf, Lb, Rb;
    DcMotorEx Lsh, Rsh;
    DcMotorEx intake;
    CRServo Lfeeder, Rfeeder;
    Servo Di;

    // ---------------- TIMERS ----------------
    private Timer pathTimer, opModeTimer;

    // ---------------- STATE ----------------
    private int pathState = 0;

    // ---------------- POSES ----------------
    private final Pose startPose = new Pose(33.916, 126.968, (2.4196));
    private final Pose scorePose = new Pose(71, 80.281, (2.3));
    private final Pose line1Pre =  new Pose(45,78, 3.070);
    private final Pose intake2Pose = new Pose(35, 86, (3.070));
    private final Pose intake3OutsidePose = new Pose(30, 88, (3.070));
    private final Pose line2pre = new Pose(47.153, 61.445, 3.070);
    private final Pose intake4Pose = new Pose(29, 60.5, (3.070));
    private final Pose intake5OutsidePose = new Pose(24, 63, (3.070));
    private final Pose intake6Pose = new Pose(18, 60, Math.toRadians(3.070));
    private final Pose line3pre = new Pose(43.750, 41.227, 3.070);
    private final Pose intake7outsidePose = new Pose(29, 39, Math.toRadians(3.070));
    private final Pose intake9Pose = new Pose(16, 36, Math.toRadians(3.070));
    private final Pose leverPose = new Pose(22.583, 60.082, (2.566));


    // ---------------- PATHS ----------------
    private PathChain score1;
    private PathChain l1Pos;
    private PathChain intakeL12;
    private PathChain intakeL13;
    private PathChain score2;
    private PathChain l2Pos;
    private PathChain intakeL21;
    private PathChain intakeL22;
    private PathChain intakeL23;
    private PathChain score3;
    private PathChain l3Pos;
    private PathChain intakeL31;
    private PathChain intakeL33;
    private PathChain score4;
    private PathChain lever;

    // ---------------- SHOOTER CONSTANTS ----------------
    private static final double CLOSE_VELOCITY =1100;
    private static final double VELOCITY_TOLERANCE = 25;
    private static final long PULSE_TIME_MS = 250;
    private static final long PULSE_GAP_MS = 150;

    // ---------------- SHOOTER STATE ----------------
    private boolean shooting = false;
    private int pulseIndex = 0;
    private int[] pulsePattern;
    private long pulseStartTime = 0;
    private boolean pulsing = false;

    // ---------------- INIT ----------------
    @Override
    public void init() {
        pathTimer = new Timer();
        opModeTimer = new Timer();
        opModeTimer.resetTimer();

        follower = Constants.createFollower(hardwareMap);
        buildPaths();
        follower.setStartingPose(startPose);
        intake = hardwareMap.get(DcMotorEx.class, "intake");
        Lsh = hardwareMap.get(DcMotorEx.class, "left_launcher");
        Rsh = hardwareMap.get(DcMotorEx.class, "right_launcher");
        Lfeeder = hardwareMap.get(CRServo.class, "left_feeder");
        Rfeeder = hardwareMap.get(CRServo.class, "right_feeder");

        Rsh.setDirection(DcMotorEx.Direction.REVERSE);
        Rfeeder.setDirection(DcMotorSimple.Direction.REVERSE);
        intake.setDirection(DcMotorSimple.Direction.REVERSE);
        Lsh.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        Rsh.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
    }

    // ---------------- LOOP ----------------
    @Override
    public void loop() {
        follower.update();
        try {
            autonomousPathUpdate();
        } catch (InterruptedException e) {
            throw new RuntimeException(e);
        }
        //updatePulseShooter();

        telemetry.addData("Path State", pathState);
        telemetry.addData("Shooter Active", shooting);
        telemetry.addData("Pulse Index", pulseIndex);
        telemetry.addData("Heading Error",follower.getHeadingError());
        telemetry.addData("angular Velocity",follower.getAngularVelocity());
        telemetry.addData("At Parametric End",follower.atParametricEnd());
        telemetry.addData("Velocity",follower.getVelocity());
        telemetry.update();

    }

    // ---------------- FSM ----------------
    public void autonomousPathUpdate() throws InterruptedException {
        switch (pathState) {
            case 0:
                follower.followPath(score1);
                if (!Schmovin()) {
                    shootAll();
                    pathState = 1;
                }
                break;
            case 1:
                intake.setPower(.75);
                follower.followPath(l1Pos);
                if (!Schmovin()) {
                    pathState = 2;
                }
                break;
            case 2:
                intake.setPower(.75);
                follower.followPath(intakeL12);
                if (!Schmovin()) {
                    pathState = 3;
                }
                break;

           case 3:
                intake.setPower(.75);
                follower.followPath(intakeL13);
                if (!Schmovin()) {
                    pathState = 4;
                }
                break;

            case 4:
                intake.setPower(0.0);
                follower.followPath(score2);
                if (!Schmovin()) {
                    shootAll();
                    pathState = 5;
                }
                break;

            case 5:
                intake.setPower(.75);
                follower.followPath(lever);
                shootAll();
                if (!Schmovin()) {

                    pathState = 6;
                }
              /*  break;
            case 5:
            follower.followPath(l2Pos);
            if (!Schmovin()) {
                pathState = 6;
            }

            case 6:
                intake.setPower(1.0);
                follower.followPath(intakeL21);
                if (!Schmovin()) {
                    pathState = 7;
                }
                break;

            case 7:
                intake.setPower(1.0);
                follower.followPath(intakeL22);
                if (!Schmovin()) {
                    pathState = 8;
                }
                break;

            case 8:
                intake.setPower(1.0);
                follower.followPath(intakeL23);
                if (!Schmovin()) {
                    pathState = 9;
                }
                break;

            case 9:
                intake.setPower(0);
                follower.followPath(score3);
                if (!Schmovin()) {
                    shootAll();
                    pathState = 10;
                }
                break;

            case 11:
                follower.followPath(l3Pos);
                if (!Schmovin()) {
                    pathState = 11;
                }

            case 12:
                intake.setPower(1.0);
                follower.followPath(intakeL31);
                if (!Schmovin()) {
                    pathState = 12;
                }
                break;
            case 13:
                intake.setPower(1.0);
                follower.followPath(intakeL33);
                if (!Schmovin()) {
                    pathState = 13;
                }
                break;

            case 14:
                intake.setPower(0);
                follower.followPath(score4);
                if (!Schmovin()) {
                    shootAll();
                    pathState = 14;
                }
                break;

             */
        }
    }

    // ---------------- SHOOTER CORE ----------------
    private void startPulsePattern(int[] pattern) {
        if (shooting) return;

        pulsePattern = pattern;
        pulseIndex = 0;
        shooting = true;
        pulsing = false;

        Lsh.setVelocity(CLOSE_VELOCITY);
        Rsh.setVelocity(CLOSE_VELOCITY);


    }

    // ---------------- FIRING MOTIFS ----------------
    // 0 = Purple (Right), 1 = Green (Left)

    public void shootAll() throws InterruptedException {
        Rsh.setVelocity(CLOSE_VELOCITY);
        Lsh.setVelocity(CLOSE_VELOCITY);
        intake.setPower(0.6);
        boolean atSpeed = false;

        //This loop is constantly asking the robot If it has reached the correct speed, if the robot hasn't then it does nothing.
        while(atSpeed == false) {
            atSpeed = Math.abs(Lsh.getVelocity() - CLOSE_VELOCITY) <= VELOCITY_TOLERANCE &&
                    Math.abs(Rsh.getVelocity() - CLOSE_VELOCITY) <= VELOCITY_TOLERANCE;
            telemetry.addData("Left shooter velocity:", Lsh.getVelocity());
            telemetry.addData("Right shooter velocity:", Rsh.getVelocity());
            telemetry.addData("firing true", atSpeed);
            telemetry.update();
            Thread.sleep(100);
        }

        Lfeeder.setPower(1.0);
        Thread.sleep(1600);

        //Stop to keep the second ball from coming up
        Lfeeder.setPower(0);

        //Wait for the flywheel to get up to speed
        Thread.sleep(500);

        //Shoot the first ball on the left
        Rfeeder.setPower(1.0);
        Thread.sleep(2000); //Change if super servo
        Rfeeder.setPower(0);

        Thread.sleep(500);

        Lfeeder.setPower(1.0);
        Thread.sleep(1600);
        Lfeeder.setPower(0.0);

        Thread.sleep(500);

        Rfeeder.setPower(1.0);
        Thread.sleep(2000); //Change if super servo
        Rfeeder.setPower(0.0);
    }

    public void shootPPG() {
        startPulsePattern(new int[]{0, 0, 1});
    }

    public void shootPGP() {
        startPulsePattern(new int[]{0, 1, 0});
    }

    public void shootGPP() {
        startPulsePattern(new int[]{1, 0, 0});
    }

    // ---------------- HELPERS ----------------
    public boolean Schmovin() {
        return Math.abs(follower.getHeadingError()) > .98 ||
                !follower.atParametricEnd() ||
                follower.getVelocity().getMagnitude() > 0.90 ||
                Math.abs(follower.getAngularVelocity()) > .90;
    }

    // ---------------- PATH BUILDER ----------------
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
        l2Pos = follower.pathBuilder()
                .addPath(new BezierLine(scorePose, line2pre))
                .setLinearHeadingInterpolation(scorePose.getHeading(), line2pre.getHeading())
                .build();

        intakeL21 = follower.pathBuilder()
                .addPath(new BezierLine(scorePose, intake4Pose))
                .setLinearHeadingInterpolation(scorePose.getHeading(), intake4Pose.getHeading())
                .build();

        intakeL22 = follower.pathBuilder()
                .addPath(new BezierLine(intake4Pose, intake5OutsidePose))
                .setLinearHeadingInterpolation(intake4Pose.getHeading(), intake5OutsidePose.getHeading())
                .build();

        intakeL23 = follower.pathBuilder()
                .addPath(new BezierLine(intake5OutsidePose, intake6Pose))
                .setLinearHeadingInterpolation(intake5OutsidePose.getHeading(), intake6Pose.getHeading())
                .build();

        score3 = follower.pathBuilder()
                .addPath(new BezierLine(intake6Pose, scorePose))
                .setLinearHeadingInterpolation(intake6Pose.getHeading(), scorePose.getHeading())
                .build();
        l3Pos = follower.pathBuilder()
                .addPath(new BezierLine(scorePose, line3pre))
                .setLinearHeadingInterpolation(scorePose.getHeading(), line3pre.getHeading())
                .build();
        intakeL31 = follower.pathBuilder()
                .addPath(new BezierLine(scorePose, intake7outsidePose))
                .setLinearHeadingInterpolation(scorePose.getHeading(), intake7outsidePose.getHeading())
                .build();
        intakeL33 = follower.pathBuilder()
                .addPath(new BezierLine(intake7outsidePose, intake9Pose ))
                .setLinearHeadingInterpolation(intake7outsidePose.getHeading(),intake9Pose .getHeading())
                .build();
         score4 = follower.pathBuilder()
                .addPath(new BezierLine(intake9Pose, scorePose ))
                .setLinearHeadingInterpolation(intake9Pose.getHeading(),scorePose .getHeading())
                .build();
        lever = follower.pathBuilder()
                .addPath(new BezierLine(scorePose, leverPose ))
                .setLinearHeadingInterpolation(scorePose.getHeading(),leverPose .getHeading())
                .build();

    }
}
