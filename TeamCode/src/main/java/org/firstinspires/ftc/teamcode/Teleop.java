package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.gamepad1;
import static org.firstinspires.ftc.teamcode.shooter.ieP;


import android.net.http.InlineExecutionProhibitedException;
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
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@TeleOp(name = "Balling Out", group = "TeleOp")
public class Teleop extends LinearOpMode {

    /// ---------------- HARDWARE ----------------
    private Follower follower;
    private DcMotorEx Lf, Rf, Lb, Rb;
    private DcMotorEx Lsh, Rsh;
    private DcMotorEx ie;
    private CRServo Lfeeder, Rfeeder;
    private GoBildaPinpointDriver odo;
    private shooter shooter;
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
    private boolean firing = false;
    private long firingStartTime;
    private ElapsedTime stateTimer = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
    private shooter.ShootingState shootingState;



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

        ie = hardwareMap.get(DcMotorEx.class, "intake");
        Lsh = hardwareMap.get(DcMotorEx.class, "left_launcher");
        Rsh = hardwareMap.get(DcMotorEx.class, "right_launcher");
        Lfeeder = hardwareMap.get(CRServo.class, "left_feeder");
        Rfeeder = hardwareMap.get(CRServo.class, "right_feeder");


        Rsh.setDirection(DcMotorEx.Direction.REVERSE);
        Rfeeder.setDirection(DcMotorSimple.Direction.REVERSE);
        ie.setDirection(DcMotorSimple.Direction.REVERSE);
        Lsh.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        Rsh.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        odo = hardwareMap.get(GoBildaPinpointDriver.class, "odo");

        Lf.setDirection(DcMotorSimple.Direction.REVERSE);
        Lb.setDirection(DcMotorSimple.Direction.REVERSE);
        Rsh.setDirection(DcMotorSimple.Direction.REVERSE);
        ie.setDirection(DcMotorSimple.Direction.REVERSE);

        /// ---------------- odometry ----------------
        odo.setOffsets(-84.0, -168.0, DistanceUnit.MM);
        odo.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        odo.setEncoderDirections(
                GoBildaPinpointDriver.EncoderDirection.FORWARD,
                GoBildaPinpointDriver.EncoderDirection.FORWARD
        );
        odo.resetPosAndIMU();

        waitForStart();


      shooter = new shooter();
        shooter.init(hardwareMap);

       odo.resetPosAndIMU();
        Pose2D startingPositon = new Pose2D(DistanceUnit.INCH, 22.583, 60.082, AngleUnit.RADIANS, 2.566);
        odo.setPosition(startingPositon);

        while (opModeIsActive()){
            follower.update();
                shooter.update();
            // live pose
            currentPose = follower.getPose();

           /* // ----- RIGHT TRIGGER: go to score pose, then shoot -----
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
//            }*/
            // DRIVER CONTROL LOCKOUT while the path runs
            /*if (ballingOut && follower.isBusy()) {
                moveRobot();
            } else if (!ballingOut && !follower.isBusy()) {
                moveRobot();
            }*/
            moveRobot();

            telemetry.addData("status", "Running");
            telemetry.addData("Pedro X", currentPose.getX());
            telemetry.addData("Pedro Y", currentPose.getY());
            telemetry.addData("Pedro Heading Deg", Math.toDegrees(currentPose.getHeading()));
            telemetry.addData("ballingOut", ballingOut);
            telemetry.addData("follower busy", follower.isBusy());
            telemetry.update();

        }

    }

    /// ---------------- controls ----------------
    public void moveRobot() {
        double forward = -gamepad1.left_stick_x;
        double strafe = -gamepad1.left_stick_y;
        double rotate = gamepad1.right_stick_x;

        if (gamepad1.optionsWasPressed()) {
            odo.resetPosAndIMU();
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
            ie.setPower(ieP);
        }else if(intakeReverseState){
            ie.setPower(-1);
        }else{
            ie.setPower(0);
        }

        // Slow mode toggle
        if (gamepad1.right_stick_button && !slowModeToggle) {
            slowMode = !slowMode;
            slowModeToggle = true;
        } else if (!gamepad1.right_stick_button) {
            slowModeToggle = false;
        }
boolean lastLeftTrigger = false;
        boolean leftTriggerPressed = gamepad1.left_trigger > 0.3;
        if (leftTriggerPressed && !lastLeftTrigger) {
            shooter.shoot();
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
        // Emergency stop
        if (gamepad1.dpad_down) {
            Lsh.setPower(0.0);
            Rsh.setPower(0.0);
            Lfeeder.setPower(0.0);
            Rfeeder.setPower(0.0);
            Rsh.setPower(0.0);
            shooter.ShotsRemaining = 0;
            shootingState = org.firstinspires.ftc.teamcode.shooter.ShootingState.IDLE;
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
