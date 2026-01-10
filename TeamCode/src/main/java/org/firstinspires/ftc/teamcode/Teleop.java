package org.firstinspires.ftc.teamcode;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@TeleOp(name = "67", group = "TeleOp")
public class Teleop extends LinearOpMode {

    /// ---------------- HARDWARE ----------------
    private Follower follower;
    private DcMotorEx Lf, Rf, Lb, Rb;
    private DcMotorEx Lsh, Rsh;
    private DcMotorEx intake;
    private CRServo Lfeeder, Rfeeder;
    private GoBildaPinpointDriver odo;

    private static final double CLOSE_VELOCITY = 1100;
    private static final double VELOCITY_TOLERANCE = 25;

    private boolean leftTriggerWasDown = false;
    private boolean ballingOut = false;
    private boolean intakeToggle = false;
    private boolean intakeReverseToggle = false;
    private boolean intakeState = false;
    private boolean intakeReverseState = false;

    private boolean slowModeToggle = false;
    private boolean slowMode = false;
    private boolean LfToggle = false;
    private boolean LfLoad = false;

    private boolean diverterToggle = false;
    private boolean diverterState = false;


    /// ---------------- POSES ----------------
    private final Pose scorePose = new Pose(71, 80.281, 2.3);

    /// ---------------- PATHS ----------------
    private PathChain score1;

    /// ---------------- LIVE POSE ----------------
    private Pose currentPose = new Pose(0, 0, 0);

    @Override
    public void runOpMode() {

        /// ---------------- Hardware ----------------
        follower = Constants.createFollower(hardwareMap);

        Lf = hardwareMap.get(DcMotorEx.class, "left_front_drive");
        Rf = hardwareMap.get(DcMotorEx.class, "right_front_drive");
        Lb = hardwareMap.get(DcMotorEx.class, "left_back_drive");
        Rb = hardwareMap.get(DcMotorEx.class, "right_back_drive");

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
        odo = hardwareMap.get(GoBildaPinpointDriver.class, "odo");

        Lf.setDirection(DcMotorSimple.Direction.REVERSE);
        Lb.setDirection(DcMotorSimple.Direction.REVERSE);
        Rsh.setDirection(DcMotorSimple.Direction.REVERSE);
        intake.setDirection(DcMotorSimple.Direction.REVERSE);

        /// ---------------- odometry ----------------
        odo.setOffsets(-84.0, -168.0, DistanceUnit.MM);
        odo.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        odo.setEncoderDirections(
                GoBildaPinpointDriver.EncoderDirection.FORWARD,
                GoBildaPinpointDriver.EncoderDirection.FORWARD
        );
        odo.resetPosAndIMU();

        waitForStart();

       /* odo.resetPosAndIMU();
        Pose2D startingPositon = new Pose2D(DistanceUnit.INCH, 22.583, 60.082, AngleUnit.RADIANS, 2.566);
        odo.setPosition(startingPositon);

        while (opModeIsActive()) {
            follower.update();

            // live pose
            currentPose = follower.getPose();

            // ----- LEFT TRIGGER: go to score pose, then shoot -----
            boolean leftTriggerDown = gamepad1.left_trigger >= 0.02;

            // on PRESS (not hold)
            if (leftTriggerDown && !leftTriggerWasDown && !ballingOut) {
                ballingOut = true;

                buildPaths();              // currentPose -> scorePose
                follower.followPath(score1);
            }
            leftTriggerWasDown = leftTriggerDown;

            // when finished, shoot once
            if (ballingOut && !follower.isBusy()) {
                ballingOut = false;
                shootAll();
            }

            // DRIVER CONTROL LOCKOUT while the path runs
            if (!ballingOut && !follower.isBusy()) {
                moveRobot();
            }

            telemetry.addData("status", "Running");
            telemetry.addData("Pedro X", currentPose.getX());
            telemetry.addData("Pedro Y", currentPose.getY());
            telemetry.addData("Pedro Heading Deg", Math.toDegrees(currentPose.getHeading()));
            telemetry.addData("ballingOut", ballingOut);
            telemetry.addData("follower busy", follower.isBusy());
            telemetry.update();*/

    }

    public void shootAll() {
        Rsh.setVelocity(CLOSE_VELOCITY);
        Lsh.setVelocity(CLOSE_VELOCITY);
        intake.setPower(0.6);
        boolean atSpeed = false;

        while (!atSpeed && opModeIsActive()) {
            atSpeed = Math.abs(Lsh.getVelocity() - CLOSE_VELOCITY) <= VELOCITY_TOLERANCE
                    && Math.abs(Rsh.getVelocity() - CLOSE_VELOCITY) <= VELOCITY_TOLERANCE;

            telemetry.addData("Left shooter velocity:", Lsh.getVelocity());
            telemetry.addData("Right shooter velocity:", Rsh.getVelocity());
            telemetry.addData("firing true", atSpeed);
            telemetry.update();
            sleep(100);
        }

        Lfeeder.setPower(1.0);
        sleep(1600);
        Lfeeder.setPower(0);

        sleep(500);

        Rfeeder.setPower(1.0);
        sleep(2000);
        Rfeeder.setPower(0);

        sleep(500);

        Lfeeder.setPower(1.0);
        sleep(1600);
        Lfeeder.setPower(0.0);

        sleep(500);

        Rfeeder.setPower(1.0);
        sleep(2000);
        Rfeeder.setPower(0.0);

        Rsh.setVelocity(0);
        Lsh.setVelocity(0);
        intake.setPower(0);
    }

    /// ---------------- controls ----------------
    public void moveRobot() {
        double forward = -gamepad1.left_stick_x;
        double strafe = -gamepad1.left_stick_y;
        double rotate = gamepad1.right_stick_x;

        if(gamepad1.left_trigger >= .02){
            shootAll();
        }

        if (gamepad1.optionsWasPressed()) {
            odo.recalibrateIMU();
        }

        Pose2D pos = odo.getPosition();
        double heading = pos.getHeading(AngleUnit.DEGREES);

        double cosAngle = Math.cos((Math.PI / 2) - Math.toRadians(heading));
        double sinAngle = Math.sin((Math.PI / 2) - Math.toRadians(heading));

        double globalStrafe = -forward * sinAngle + strafe * cosAngle;
        double globalForward = forward * cosAngle + strafe * sinAngle;

        double[] newWheelSpeeds = new double[4];

        newWheelSpeeds[0] = globalForward + globalStrafe + rotate;
        newWheelSpeeds[1] = globalForward - globalStrafe - rotate;
        newWheelSpeeds[2] = globalForward - globalStrafe + rotate;
        newWheelSpeeds[3] = globalForward + globalStrafe - rotate;

        Lf.setPower(newWheelSpeeds[0]);
        Rf.setPower(newWheelSpeeds[1]);
        Lb.setPower(newWheelSpeeds[2]);
        Rb.setPower(newWheelSpeeds[3]);

        telemetry.addData("Forward Speed : ", globalForward);
        telemetry.addData("Strafe Speed :", globalStrafe);
        // Intake toggle
        if (gamepad1.left_stick_button && !intakeToggle && !intakeReverseState) {
            intakeState = !intakeState;
            intakeToggle = true;
        } else if (!gamepad1.left_stick_button) {
            intakeToggle = false;
        }

        // Intake reverse toggle
        if (gamepad1.dpad_left && !intakeReverseToggle && !intakeState) {
            intakeReverseState = !intakeReverseState;
            intakeReverseToggle = true;
        } else if (!gamepad1.dpad_left) {
            intakeReverseToggle = false;
        }

        if(intakeState){
            intake.setPower(1);
        }else if(intakeReverseState){
            intake.setPower(-1);
        }else{
            intake.setPower(0);
        }

        // Slow mode toggle
        if (gamepad1.right_stick_button && !slowModeToggle) {
            slowMode = !slowMode;
            slowModeToggle = true;
        } else if (!gamepad1.right_stick_button) {
            slowModeToggle = false;
        }

        // feeder toggle (L1/R1)
        if (gamepad1.left_bumper){
            Lfeeder.setPower(1.0);
        } else if (gamepad1.left_bumper){
            Lfeeder.setPower(0.0);
        }
        if (gamepad1.right_bumper){
            Rfeeder.setPower(1.0);
        } else if (gamepad1.right_bumper){
            Rfeeder.setPower(0.0);
        }
    }

    /// ---------------- PATH BUILDER ----------------
    public void buildPaths() {
        score1 = follower.pathBuilder()
                .addPath(new BezierLine(currentPose, scorePose))
                .setLinearHeadingInterpolation(currentPose.getHeading(), scorePose.getHeading())
                .build();
    }
}
