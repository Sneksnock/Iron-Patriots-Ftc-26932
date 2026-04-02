package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.teamcode.shooter.ieP;
import static org.firstinspires.ftc.teamcode.shooter.lift;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.pedroPathing.MatchPoseMemory;

@TeleOp(name = "Balling Out", group = "TeleOp")
public class Blue_Teleop extends LinearOpMode {

    private LimelightShooterMechanism vision;
    private Follower follower;
    private DcMotorEx Lf, Rf, Lb, Rb;
    private DcMotorEx ie;
    private GoBildaPinpointDriver odo;
    private shooter shooter;

    private boolean ballingOut = false;
    private boolean intakeToggle = false;
    private boolean intakeReverseToggle = false;
    private boolean intakeState = false;
    private boolean intakeReverseState = false;
    private boolean liftUp = false;
    private boolean lastY = false;
    private boolean slowModeToggle = false;
    private boolean slowMode = false;
    private boolean lastLeftBumper = false;
    private boolean lastRightBumper = false;
    private boolean lastLeftTriggerPressed = false;
    private boolean lastOptionsPressed = false;
    private boolean lastAPressed = false;
    private boolean lastBackPressed = false;

    private boolean visionAimActive = false;
    private boolean visionShotRequested = false;

    // ---------- VISION / HEADING TUNING ----------
    public static double VISION_TRANSLATION_SCALE = 0.45;
    public static boolean REQUIRE_TARGET_FOR_VISION_SHOT = true;

    public static double HEADING_HOLD_KP = 1.8;
    public static double HEADING_HOLD_DEADBAND_DEG = 1.0;
    public static double HEADING_HOLD_MAX = 0.45;

    public static double VISION_FIRE_TX_TOLERANCE = 0.9;
    public static double MANUAL_FIRE_TX_TOLERANCE = 1.25;

    public static boolean BLOCK_SHOTS_WHEN_MISALIGNED = true;
    public static double MANUAL_ROTATE_DEADBAND = 0.08;

    private double headingTargetRad = 0.0;

    private final Pose defaultTeleopStartPose = new Pose(22.583, 60.082, 2.566);
    private final Pose scorePose = new Pose(71, 80.281, 2.3);
    private PathChain score1;
    private Pose currentPose = new Pose(0, 0, 0);

    @Override
    public void runOpMode() {

        follower = Constants.createFollower(hardwareMap);

        Lf = hardwareMap.get(DcMotorEx.class, "left_front_drive");
        Rf = hardwareMap.get(DcMotorEx.class, "right_front_drive");
        Lb = hardwareMap.get(DcMotorEx.class, "left_back_drive");
        Rb = hardwareMap.get(DcMotorEx.class, "right_back_drive");

        ie = hardwareMap.get(DcMotorEx.class, "intake");

        Lf.setDirection(DcMotorSimple.Direction.REVERSE);
        Lb.setDirection(DcMotorSimple.Direction.REVERSE);
        ie.setDirection(DcMotorSimple.Direction.REVERSE);

        odo = hardwareMap.get(GoBildaPinpointDriver.class, "odo");

        shooter = new shooter();
        shooter.init(hardwareMap);

        LimelightShooterMechanism.RED_ALLIANCE = false;

        vision = new LimelightShooterMechanism(
                hardwareMap,
                telemetry,
                LimelightShooterMechanism.BLUE_PIPELINE_INDEX
        );

        odo.setOffsets(-84.0, -168.0, DistanceUnit.MM);
        odo.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        odo.setEncoderDirections(
                GoBildaPinpointDriver.EncoderDirection.FORWARD,
                GoBildaPinpointDriver.EncoderDirection.FORWARD
        );

        Pose teleopStartPose = MatchPoseMemory.getPoseOrDefault(defaultTeleopStartPose);
        follower.setStartingPose(teleopStartPose);
        headingTargetRad = teleopStartPose.getHeading();

        telemetry.addLine("Initialized");
        telemetry.addData("Using Saved Auto Pose", MatchPoseMemory.hasPose());
        telemetry.addData("Start X", teleopStartPose.getX());
        telemetry.addData("Start Y", teleopStartPose.getY());
        telemetry.addData("Start Heading Deg", Math.toDegrees(teleopStartPose.getHeading()));
        telemetry.addLine("LT = vision aim + auto fire when aligned");
        telemetry.addLine("LB = close shot only when aligned");
        telemetry.addLine("RB = far shot only when aligned");
        telemetry.addLine("Back/Share = toggle RED/BLUE pipeline");
        telemetry.addLine("Dpad Down = cancel shot + cancel vision aim");
        telemetry.update();

        waitForStart();
        if (isStopRequested()) return;

        odo.resetPosAndIMU();
        odo.setPosition(new Pose2D(
                DistanceUnit.INCH,
                teleopStartPose.getX(),
                teleopStartPose.getY(),
                AngleUnit.RADIANS,
                teleopStartPose.getHeading()
        ));

        vision.start();

        while (opModeIsActive()) {
            follower.update();

            handlePipelineToggle();

            vision.update(
                    gamepad1.dpad_up,
                    false
            );

            shooter.setVisionVelocity(vision.getSuggestedVelocity());

            moveRobot();

            shooter.update();

            Pose2D odoPose = odo.getPosition();
            currentPose = new Pose(
                    odoPose.getX(DistanceUnit.INCH),
                    odoPose.getY(DistanceUnit.INCH),
                    odoPose.getHeading(AngleUnit.RADIANS)
            );

            LLResult result = vision.getLimelightResult();
            boolean hasTarget = result != null && result.isValid();

            telemetry.addData("status", "Running");
            telemetry.addData("Saved Auto Pose", MatchPoseMemory.hasPose());

            telemetry.addData("Odo X", currentPose.getX());
            telemetry.addData("Odo Y", currentPose.getY());
            telemetry.addData("Odo Heading Deg", Math.toDegrees(currentPose.getHeading()));
            telemetry.addData("Heading Target Deg", Math.toDegrees(headingTargetRad));
            telemetry.addData("Heading Error Deg", getHeadingErrorDeg(currentPose.getHeading(), headingTargetRad));

            telemetry.addData("ballingOut", ballingOut);
            telemetry.addData("shooter busy", shooter.isBusy());
            telemetry.addData("shooter state", shooter.getState());
            telemetry.addData("shots remaining", shooter.ShotsRemaining);

            telemetry.addData("Vision Aim Active", visionAimActive);
            telemetry.addData("Vision Shot Requested", visionShotRequested);
            telemetry.addData("Alliance Pipeline", LimelightShooterMechanism.RED_ALLIANCE ? "RED" : "BLUE");

            telemetry.addData("LL Valid", hasTarget);
            telemetry.addData("LL ta", result != null ? result.getTa() : 0.0);
            telemetry.addData("LL tx", result != null ? result.getTx() : 0.0);
            telemetry.addData("LL ty", result != null ? result.getTy() : 0.0);

            telemetry.addData("Vision Suggested Velocity", vision.getSuggestedVelocity());
            telemetry.addData("Can Fire Now", canFireNow(hasTarget, result, VISION_FIRE_TX_TOLERANCE));

            if (BLOCK_SHOTS_WHEN_MISALIGNED && !canFireNow(hasTarget, result, MANUAL_FIRE_TX_TOLERANCE)) {
                telemetry.addLine("SHOT BLOCKED: not aligned");
            }

            telemetry.update();
        }

        vision.stop();
    }

    private void handlePipelineToggle() {
        boolean backPressed = gamepad1.back || gamepad1.share;

        if (backPressed && !lastBackPressed) {
            LimelightShooterMechanism.RED_ALLIANCE = !LimelightShooterMechanism.RED_ALLIANCE;
            vision.updateAlliancePipeline();
        }

        lastBackPressed = backPressed;
    }

    public void moveRobot() {
        double forward = -gamepad1.left_stick_x;
        double strafe = -gamepad1.left_stick_y;
        double rotateInput = gamepad1.right_stick_x;
        double rotate;

        boolean optionsPressed = gamepad1.options;
        if (optionsPressed && !lastOptionsPressed) {
            Pose2D current = odo.getPosition();
            headingTargetRad = current.getHeading(AngleUnit.RADIANS);
        }
        lastOptionsPressed = optionsPressed;

        Pose2D pos = odo.getPosition();
        double headingDeg = pos.getHeading(AngleUnit.DEGREES);
        double headingRad = pos.getHeading(AngleUnit.RADIANS);

        LLResult result = vision.getLimelightResult();
        boolean hasTarget = result != null && result.isValid();

        boolean leftTriggerPressed = gamepad1.left_trigger > 0.3;
        if (leftTriggerPressed && !lastLeftTriggerPressed) {
            if (!REQUIRE_TARGET_FOR_VISION_SHOT || hasTarget) {
                visionAimActive = true;
                visionShotRequested = false;
            }
        }
        lastLeftTriggerPressed = leftTriggerPressed;

        boolean aPressed = gamepad1.a;
        if (aPressed && !lastAPressed && !shooter.isBusy()) {
            if (!BLOCK_SHOTS_WHEN_MISALIGNED || canFireNow(hasTarget, result, MANUAL_FIRE_TX_TOLERANCE)) {
                shooter.requestVisionShot();
            }
        }
        lastAPressed = aPressed;

        if (gamepad1.dpad_down) {
            shooter.cancelShot();
            ie.setPower(0.0);
            visionAimActive = false;
            visionShotRequested = false;
            headingTargetRad = headingRad;
        }

        if (visionAimActive) {
            if (hasTarget) {
                forward *= VISION_TRANSLATION_SCALE;
                strafe *= VISION_TRANSLATION_SCALE;

                // Use known robot heading + Limelight tx to build a heading target
                headingTargetRad = wrapAngleRad(headingRad - Math.toRadians(result.getTx()));

                if (canFireNow(hasTarget, result, VISION_FIRE_TX_TOLERANCE) && !visionShotRequested && !shooter.isBusy()) {
                    shooter.requestVisionShot();
                    visionShotRequested = true;
                }

                if (visionShotRequested && !shooter.isBusy()) {
                    visionAimActive = false;
                    visionShotRequested = false;
                    headingTargetRad = headingRad;
                }
            } else {
                visionAimActive = false;
                visionShotRequested = false;
            }
        }

        boolean driverIsTurning = Math.abs(rotateInput) > MANUAL_ROTATE_DEADBAND;

        if (driverIsTurning) {
            rotate = rotateInput;
            headingTargetRad = headingRad;
        } else {
            rotate = getHeadingHoldCorrection(headingRad, headingTargetRad);
        }

        double cosAngle = Math.cos((Math.PI / 2) - Math.toRadians(headingDeg));
        double sinAngle = Math.sin((Math.PI / 2) - Math.toRadians(headingDeg));

        double globalStrafe = -forward * sinAngle + strafe * cosAngle;
        double globalForward = forward * cosAngle + strafe * sinAngle;

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

        double max = Math.max(
                1.0,
                Math.max(
                        Math.abs(newWheelSpeeds[0]),
                        Math.max(
                                Math.abs(newWheelSpeeds[1]),
                                Math.max(Math.abs(newWheelSpeeds[2]), Math.abs(newWheelSpeeds[3]))
                        )
                )
        );

        Lf.setPower(newWheelSpeeds[0] / max);
        Rf.setPower(newWheelSpeeds[1] / max);
        Lb.setPower(newWheelSpeeds[2] / max);
        Rb.setPower(newWheelSpeeds[3] / max);

        if (intakeState) {
            ie.setVelocity(ieP);
        } else if (intakeReverseState) {
            ie.setPower(-1);
        } else if (!shooter.isBusy()) {
            ie.setPower(0);
        }

        if (gamepad1.left_stick_button && !intakeToggle && !intakeReverseState) {
            intakeState = !intakeState;
            intakeToggle = true;
        } else if (!gamepad1.left_stick_button) {
            intakeToggle = false;
        }

        if (gamepad1.dpad_left && !intakeReverseToggle && !intakeState) {
            intakeReverseState = !intakeReverseState;
            intakeReverseToggle = true;
        } else if (!gamepad1.dpad_left) {
            intakeReverseToggle = false;
        }

        boolean leftBumperPressed = gamepad1.left_bumper;
        boolean rightBumperPressed = gamepad1.right_bumper;

        if (leftBumperPressed && !lastLeftBumper && !shooter.isBusy()) {
            if (!BLOCK_SHOTS_WHEN_MISALIGNED || canFireNow(hasTarget, result, MANUAL_FIRE_TX_TOLERANCE)) {
                shooter.requestCloseShot();
            }
        }

        if (rightBumperPressed && !lastRightBumper && !shooter.isBusy()) {
            if (!BLOCK_SHOTS_WHEN_MISALIGNED || canFireNow(hasTarget, result, MANUAL_FIRE_TX_TOLERANCE)) {
                shooter.requestFarShot();
            }
        }

        lastLeftBumper = leftBumperPressed;
        lastRightBumper = rightBumperPressed;

        boolean yPressed = gamepad1.y;

        if (yPressed && !lastY) {
            liftUp = !liftUp;
        }

        lastY = yPressed;

        if (liftUp) {
            lift.setPosition(0.8);
        } else {
            lift.setPosition(0.2);
        }
    }

    private boolean canFireNow(boolean hasTarget, LLResult result, double txTolerance) {
        if (!hasTarget || result == null) {
            return false;
        }
        return Math.abs(result.getTx()) <= txTolerance;
    }

    private double getHeadingHoldCorrection(double currentHeadingRad, double targetHeadingRad) {
        double errorDeg = getHeadingErrorDeg(currentHeadingRad, targetHeadingRad);

        if (Math.abs(errorDeg) <= HEADING_HOLD_DEADBAND_DEG) {
            return 0.0;
        }

        double turn = (errorDeg / 180.0) * HEADING_HOLD_KP;

        if (turn > HEADING_HOLD_MAX) turn = HEADING_HOLD_MAX;
        if (turn < -HEADING_HOLD_MAX) turn = -HEADING_HOLD_MAX;

        return turn;
    }

    private double getHeadingErrorDeg(double currentHeadingRad, double targetHeadingRad) {
        return Math.toDegrees(wrapAngleRad(targetHeadingRad - currentHeadingRad));
    }

    private double wrapAngleRad(double angle) {
        while (angle <= -Math.PI) angle += 2.0 * Math.PI;
        while (angle > Math.PI) angle -= 2.0 * Math.PI;
        return angle;
    }

    public void buildPaths() {
        score1 = follower.pathBuilder()
                .addPath(new BezierLine(currentPose, scorePose))
                .setLinearHeadingInterpolation(currentPose.getHeading(), scorePose.getHeading())
                .build();
    }
}