package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.teamcode.shooter.ieP;

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

@TeleOp(name = "Balling Out", group = "TeleOp")
public class Teleop extends LinearOpMode {

    /// ---------------- HARDWARE ----------------
     private FOuR_EYeS vision;
    private Follower follower;
    private DcMotorEx Lf, Rf, Lb, Rb;
    private DcMotorEx Lsh, Rsh;
    private DcMotorEx ie;
    private CRServo Lfeeder, Rfeeder;
    private GoBildaPinpointDriver odo;
    private shooter shooter;

    /// ---------------- DRIVER STATE ----------------
    private boolean ballingOut = false;
    private boolean intakeToggle = false;
    private boolean intakeReverseToggle = false;
    private boolean intakeState = false;
    private boolean intakeReverseState = false;

    private boolean slowModeToggle = false;
    private boolean slowMode = false;

    /// ---------------- EDGE DETECTION ----------------
    private boolean lastLeftTriggerPressed = false;
    private boolean lastRightTriggerPressed = false;

    /// ---------------- POSES ----------------
    /// ---------- TODO: SHOOTING POSES ----------
    /// Replace this placeholder / current pose with your final tested teleop shooting poses.
    /// Blue close should come from Blue_Side_Pedro scorePose
    /// Blue far should come from Far_Side_Pedro scorePose
    /// Red close should come from Red_Side_Pedro scorePose
    /// Red far should come from Far_Side_Pedro_red scorePose
    private final Pose scorePose = new Pose(71, 80.281, 2.3);

    /// ---------------- PATHS ----------------
    private PathChain score1;

    /// ---------------- LIVE POSE ----------------
    private Pose currentPose = new Pose(0, 0, 0);

    /// ---------------- TODO: LIMELIGHT ----------
    /// When FOuR_EYeS is integrated, add:
    /// private FOuR_EYeS vision;
    ///
    /// Then use it for:
    /// - AprilTag heading lock
    /// - smart shot distance
    /// - alliance tag filtering

    @Override
    public void runOpMode() {

        /// ---------------- HARDWARE ----------------
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
        vision = new FOuR_EYeS();
        vision.init(hardwareMap);

        Lf.setDirection(DcMotorSimple.Direction.REVERSE);
        Lb.setDirection(DcMotorSimple.Direction.REVERSE);
        Rsh.setDirection(DcMotorSimple.Direction.REVERSE);
        ie.setDirection(DcMotorSimple.Direction.REVERSE);

        /// ---------------- ODOMETRY ----------------
        odo.setOffsets(-84.0, -168.0, DistanceUnit.MM);
        odo.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        odo.setEncoderDirections(
                GoBildaPinpointDriver.EncoderDirection.FORWARD,
                GoBildaPinpointDriver.EncoderDirection.FORWARD
        );
        odo.resetPosAndIMU();

        shooter = new shooter();
        shooter.init(hardwareMap);

        /// ---------------- TODO: LIMELIGHT INIT ----------
        /// vision = new FOuR_EYeS();
        /// vision.init(hardwareMap);

        waitForStart();

        odo.resetPosAndIMU();
        Pose2D startingPositon = new Pose2D(
                DistanceUnit.INCH,
                22.583,
                60.082,
                AngleUnit.RADIANS,
                2.566
        );
        odo.setPosition(startingPositon);

        while (opModeIsActive()) {
            follower.update();
            shooter.update();
            vision.update();

            /// ---------------- TODO: LIMELIGHT UPDATE ----------
            /// vision.update();

            currentPose = follower.getPose();

            /// ---------- TODO: SCORE ASSIST ----------
            /// This old score-pose path block is still commented out.
            /// Later, this can be replaced with:
            /// - nearest shooting pose selection
            /// - move to pose
            /// - limelight aim
            /// - smart fire

            /*
            boolean leftTriggerDown = gamepad1.left_trigger >= 0.02;

            if (leftTriggerDown && !leftTriggerWasDown && !ballingOut) {
                ballingOut = true;

                buildPaths();
                follower.followPath(score1);
            }
            leftTriggerWasDown = leftTriggerDown;

            if (ballingOut && !follower.isBusy()) {
                ballingOut = false;
                shootAll();
            }
            */

            moveRobot();

            telemetry.addData("status", "Running");
            telemetry.addData("Pedro X", currentPose.getX());
            telemetry.addData("Pedro Y", currentPose.getY());
            telemetry.addData("Pedro Heading Deg", Math.toDegrees(currentPose.getHeading()));
            telemetry.addData("ballingOut", ballingOut);
            telemetry.addData("follower busy", follower.isBusy());
            telemetry.addData("shooter busy", shooter.isBusy());
            telemetry.addData("shooter state", shooter.getState());
            telemetry.addData("shots remaining", shooter.ShotsRemaining);
            telemetry.addData("Vision Target", vision.hasValidTarget());
            telemetry.addData("Vision Yaw Error", vision.getYawErrorDeg());
            telemetry.addData("Vision Distance", vision.getDistanceInches());
            telemetry.update();
        }
    }

    /// ---------------- CONTROLS ----------------
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

        /// ---------------- SLOW MODE ----------------
        if (gamepad1.right_stick_button && !slowModeToggle) {
            slowMode = !slowMode;
            slowModeToggle = true;
        } else if (!gamepad1.right_stick_button) {
            slowModeToggle = false;
        }

        if (slowMode) {
            globalForward *= 0.4;
            globalStrafe *= 0.4;
            rotate *= 0.4;
        }

        double[] newWheelSpeeds = new double[4];
        newWheelSpeeds[0] = globalForward + globalStrafe + rotate;
        newWheelSpeeds[1] = globalForward - globalStrafe - rotate;
        newWheelSpeeds[2] = globalForward - globalStrafe + rotate;
        newWheelSpeeds[3] = globalForward + globalStrafe - rotate;

        Lf.setPower(newWheelSpeeds[0]);
        Rf.setPower(newWheelSpeeds[1]);
        Lb.setPower(newWheelSpeeds[2]);
        Rb.setPower(newWheelSpeeds[3]);

        telemetry.addData("Forward Speed", globalForward);
        telemetry.addData("Strafe Speed", globalStrafe);

        /// ---------------- INTAKE TOGGLE ----------------
        if (gamepad1.left_stick_button && !intakeToggle && !intakeReverseState) {
            intakeState = !intakeState;
            intakeToggle = true;
        } else if (!gamepad1.left_stick_button) {
            intakeToggle = false;
        }

        /// ---------------- INTAKE REVERSE TOGGLE ----------------
        if (gamepad1.dpad_left && !intakeReverseToggle && !intakeState) {
            intakeReverseState = !intakeReverseState;
            intakeReverseToggle = true;
        } else if (!gamepad1.dpad_left) {
            intakeReverseToggle = false;
        }

        if (intakeState) {
            ie.setPower(ieP);
        } else if (intakeReverseState) {
            ie.setPower(-1);
        } else {
            ie.setPower(0);
        }

        /// ---------------- SHOOTER REQUESTS ----------------
        boolean leftTriggerPressed = gamepad1.left_trigger > 0.3;
        if (leftTriggerPressed && !lastLeftTriggerPressed) {
            shooter.requestCloseShot();
        }
        lastLeftTriggerPressed = leftTriggerPressed;

        boolean rightTriggerPressed = gamepad1.right_trigger > 0.3;
        if (rightTriggerPressed && !lastRightTriggerPressed) {
            shooter.requestFarShot();
        }
        lastRightTriggerPressed = rightTriggerPressed;

        /// ---------------- MANUAL FEEDER CONTROL ----------------
        /// These are manual overrides. They should only be used when not auto-shooting.
        if (gamepad1.left_bumper) {
            shooter.setLeftFeederManual(1.0);
        } else if (!shooter.isBusy()) {
            shooter.setLeftFeederManual(0.0);
        }

        if (gamepad1.right_bumper) {
            shooter.setRightFeederManual(1.0);
        } else if (!shooter.isBusy()) {
            shooter.setRightFeederManual(0.0);
        }

        /// ---------------- EMERGENCY STOP ----------------
        if (gamepad1.dpad_down) {
            shooter.cancelShot();
            ie.setPower(0.0);
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