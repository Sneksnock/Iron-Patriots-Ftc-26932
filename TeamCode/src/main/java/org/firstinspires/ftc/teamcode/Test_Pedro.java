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
public class Test_Pedro extends OpMode {

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
    private final Pose startPose = new Pose(20.6, 121, Math.toRadians(135));
    private final Pose scorePose = new Pose(69, 73, Math.toRadians(135));
    private final Pose intake2Pose = new Pose(55, 53, Math.toRadians(180));
    private final Pose intake3OutsidePose = new Pose(23, 86, Math.toRadians(180));
    private final Pose intake4Pose = new Pose(29, 60.5, Math.toRadians(180));
    private final Pose intake5OutsidePose = new Pose(24, 63, Math.toRadians(180));
    private final Pose intake6Pose = new Pose(18, 60, Math.toRadians(180));
    private final Pose intake7outsidePose = new Pose(29, 39, Math.toRadians(180));
    private final Pose intake9Pose = new Pose(16, 36, Math.toRadians(180));
    private final Pose leverPose = new Pose(19, 72, Math.toRadians(90));


    // ---------------- PATHS ----------------
    private PathChain score1;
    private PathChain intakeL12;
    private PathChain intakeL13;
    private PathChain score2;
    private PathChain intakeL21;
    private PathChain intakeL22;
    private PathChain intakeL23;
    private PathChain score3;
    private PathChain intakeL31;
    private PathChain intakeL33;
    private PathChain score4;
    private PathChain lever;

    // ---------------- SHOOTER CONSTANTS ----------------
    private static final double CLOSE_VELOCITY = 1150;
    private static final double VELOCITY_TOLERANCE = 75;
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
                intake.setPower(1.0);
                follower.followPath(intakeL12);
                if (!Schmovin()) {
                    pathState = 2;
                }
                break;

            case 2:
                intake.setPower(1.0);
                follower.followPath(intakeL13);
                if (!Schmovin()) {
                    pathState = 3;
                }
                break;

            case 3:
                intake.setPower(0);
                follower.followPath(score2);
                if (!Schmovin()) {
                    shootAll();
                    pathState = 4;
                }
                break;

            case 4:
                intake.setPower(1.0);
                follower.followPath(intakeL21);
                if (!Schmovin()) {
                    pathState = 5;
                }
                break;

            case 5:
                intake.setPower(1.0);
                follower.followPath(intakeL22);
                if (!Schmovin()) {
                    pathState = 6;
                }
                break;

            case 6:
                intake.setPower(1.0);
                follower.followPath(intakeL23);
                if (!Schmovin()) {
                    pathState = 7;
                }
                break;

            case 7:
                intake.setPower(0);
                follower.followPath(score3);
                if (!Schmovin()) {
                    shootAll();
                    pathState = 8;
                }
                break;

            case 8:
                intake.setPower(1.0);
                follower.followPath(intakeL31);
                if (!Schmovin()) {
                    pathState = 9;
                }
                break;
            case 9:
                intake.setPower(1.0);
                follower.followPath(intakeL33);
                if (!Schmovin()) {
                    pathState = 10;
                }
                break;

            case 10:
                intake.setPower(0);
                follower.followPath(score4);
                if (!Schmovin()) {
                    shootAll();
                    pathState = 11;
                }
                break;

            case 11:
                follower.followPath(lever);
                if (!Schmovin()) {
                    pathState = 12;
                }
                break;
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

    private void updatePulseShooter() {
        if (!shooting) return;


        double lv = Lsh.getVelocity();
        double rv = Rsh.getVelocity();

        boolean atSpeed = Math.abs(lv - CLOSE_VELOCITY) <= VELOCITY_TOLERANCE &&
                        Math.abs(rv - CLOSE_VELOCITY) <= VELOCITY_TOLERANCE;

        if (!atSpeed) return;

        long now = System.currentTimeMillis();

        if (!pulsing) {
            pulsing = true;
            pulseStartTime = now;

            if (pulsePattern[pulseIndex] == 0) {
                Rfeeder.setPower(1.0);   // Purple
            } else {
                Lfeeder.setPower(1.0);   // Green
            }
        }

        if (pulsing && now - pulseStartTime >= PULSE_TIME_MS) {
            Lfeeder.setPower(0);
            Rfeeder.setPower(0);
            pulsing = false;
            pulseIndex++;

            if (pulseIndex >= pulsePattern.length) {
                Lsh.setVelocity(0);
                Rsh.setVelocity(0);
                shooting = false;
            } else {
                pulseStartTime = now + PULSE_GAP_MS;
            }
        }
    }

    // ---------------- FIRING MOTIFS ----------------
    // 0 = Purple (Right), 1 = Green (Left)

    public void shootAll() throws InterruptedException {
        Rsh.setVelocity(CLOSE_VELOCITY);
        Lsh.setVelocity(CLOSE_VELOCITY);
        boolean atSpeed = false;

        /*
        //This loop is constantly asking the robot If it has reached the correct speed, if the robot hasn't then it does nothing.
        while(atSpeed == false){
            i++;
            atSpeed = Math.abs(Lsh.getVelocity() - CLOSE_VELOCITY) <= VELOCITY_TOLERANCE &&
                    Math.abs(Rsh.getVelocity() - CLOSE_VELOCITY) <= VELOCITY_TOLERANCE;
            telemetry.addData("Left shooter velocity:", Lsh.getVelocity());
            telemetry.addData("Right shooter velocity:", Rsh.getVelocity());
            telemetry.addData("firing true", atSpeed);
            telemetry.update();
            Thread.sleep(100);
        }*/
        Thread.sleep(2000);
        Lfeeder.setPower(1.0);
        Thread.sleep(750);
        Rfeeder.setPower(1.0);
        Thread.sleep(500);
        intake.setPower(.5);
        Thread.sleep(3000);
        Lfeeder.setPower(0);
        Rfeeder.setPower(0);
        intake.setPower(.5);
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
        return Math.abs(follower.getHeadingError()) > .09  ||
                !follower.atParametricEnd() ||
                follower.getVelocity().getMagnitude() > 0.3 ||
                Math.abs(follower.getAngularVelocity()) > .05;
    }

    // ---------------- PATH BUILDER ----------------
    public void buildPaths() {

        score1 = follower.pathBuilder()
                .addPath(new BezierLine(startPose, scorePose))
                .setLinearHeadingInterpolation(startPose.getHeading(), scorePose.getHeading())
                .build();

        intakeL12 = follower.pathBuilder()
                .addPath(new BezierLine(scorePose, intake2Pose))
                .setLinearHeadingInterpolation(scorePose.getHeading(), intake2Pose.getHeading())
                .build();

        intakeL13 = follower.pathBuilder()
                .addPath(new BezierLine(intake2Pose, intake3OutsidePose))
                .setLinearHeadingInterpolation(intake2Pose.getHeading(), intake3OutsidePose.getHeading())
                .build();

        score2 = follower.pathBuilder()
                .addPath(new BezierLine(intake3OutsidePose, scorePose))
                .setLinearHeadingInterpolation(intake3OutsidePose.getHeading(), scorePose.getHeading())
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
