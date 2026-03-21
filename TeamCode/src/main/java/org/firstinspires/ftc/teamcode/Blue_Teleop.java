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

    private boolean lastLeftTriggerPressed = false;
    private boolean lastOptionsPressed = false;
    private boolean lastAPressed = false;

    private boolean visionAimActive = false;
    private boolean visionShotRequested = false;

    public static double VISION_TURN_KP = 0.05;
    public static double VISION_TURN_DEADBAND = 0.25;
    public static double VISION_TURN_MAX = 0.45;
    public static double VISION_AIM_TOLERANCE = 2.0;
    public static double VISION_TRANSLATION_SCALE = 0.45;
    public static boolean REQUIRE_TARGET_FOR_VISION_SHOT = false;

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
        odo.resetPosAndIMU();

        telemetry.addLine("Initialized");
        telemetry.addLine("LT = vision aim + vision shot");
        telemetry.addLine("A = manual vision shot test");
        telemetry.addLine("Dpad Down = cancel shot");
        telemetry.update();

        waitForStart();

        odo.resetPosAndIMU();
        Pose2D startingPosition = new Pose2D(
                DistanceUnit.INCH,
                22.583,
                60.082,
                AngleUnit.RADIANS,
                2.566
        );
        odo.setPosition(startingPosition);

        vision.start();

        while (opModeIsActive()) {
            follower.update();

            vision.update(
                    gamepad1.dpad_up,
                    false
            );

            shooter.setVisionVelocity(vision.getSuggestedVelocity());

            moveRobot();

            shooter.update();

            currentPose = follower.getPose();

            LLResult result = vision.getLimelightResult();
            boolean hasTarget = result != null && result.isValid();

            telemetry.addData("status", "Running");
            telemetry.addData("Pedro X", currentPose.getX());
            telemetry.addData("Pedro Y", currentPose.getY());
            telemetry.addData("Pedro Heading Deg", Math.toDegrees(currentPose.getHeading()));

            telemetry.addData("ballingOut", ballingOut);
            telemetry.addData("follower busy", follower.isBusy());

            telemetry.addData("shooter busy", shooter.isBusy());
            telemetry.addData("shooter state", shooter.getState());
            telemetry.addData("shots remaining", shooter.ShotsRemaining);

            telemetry.addData("Vision Aim Active", visionAimActive);
            telemetry.addData("Vision Shot Requested", visionShotRequested);

            telemetry.addData("LL Result Null", result == null);
            telemetry.addData("LL Valid", hasTarget);
            telemetry.addData("LL ta", result != null ? result.getTa() : 0.0);
            telemetry.addData("LL tx", result != null ? result.getTx() : 0.0);
            telemetry.addData("LL ty", result != null ? result.getTy() : 0.0);

            telemetry.addData("Vision Suggested Velocity", vision.getSuggestedVelocity());
            telemetry.addData("Shooter Vision Velocity", shooter.getVisionVelocity());
            telemetry.addData("Vision Heading Correction", hasTarget ? getVisionTurnCorrection(result.getTx()) : 0.0);

            telemetry.update();
        }

        vision.stop();
    }

    public void moveRobot() {
        double forward = -gamepad1.left_stick_x;
        double strafe = -gamepad1.left_stick_y;
        double rotate = gamepad1.right_stick_x;

        boolean optionsPressed = gamepad1.options;
        if (optionsPressed && !lastOptionsPressed) {
            odo.resetPosAndIMU();
        }
        lastOptionsPressed = optionsPressed;

        Pose2D pos = odo.getPosition();
        double heading = pos.getHeading(AngleUnit.DEGREES);

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
            shooter.requestVisionShot();
        }
        lastAPressed = aPressed;

        if (gamepad1.dpad_down) {
            shooter.cancelShot();
            ie.setPower(0.0);
            visionAimActive = false;
            visionShotRequested = false;
        }

        if (visionAimActive) {
            if (hasTarget) {
                forward *= VISION_TRANSLATION_SCALE;
                strafe *= VISION_TRANSLATION_SCALE;

                if (Math.abs(result.getTx()) <= VISION_AIM_TOLERANCE && !visionShotRequested && !shooter.isBusy()) {
                    shooter.requestVisionShot();
                    visionShotRequested = true;
                }

                if (visionShotRequested && !shooter.isBusy()) {
                    visionAimActive = false;
                    visionShotRequested = false;
                }
            } else if (REQUIRE_TARGET_FOR_VISION_SHOT) {
                visionAimActive = false;
                visionShotRequested = false;
            }
        }

        double cosAngle = Math.cos((Math.PI / 2) - Math.toRadians(heading));
        double sinAngle = Math.sin((Math.PI / 2) - Math.toRadians(heading));

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

        if (!shooter.isBusy()) {
            if (gamepad1.left_bumper) {
                shooter.setLeftFeederManual(1.0);
            } else {
                shooter.setLeftFeederManual(0.0);
            }

    /*       if (gamepad1.yWasPressed()) {
              lift.setPower(0.0);
           } else if (gamepad1.yWasPressed()){
            lift.setPower(-1.0);
         } */

            boolean yPressed = gamepad1.y;

            if (yPressed && !lastY) {
                liftUp = !liftUp; // toggle state
            }

            lastY = yPressed;

            if (liftUp) {
                lift.setPosition(0.8);
            } else {
                lift.setPosition(0.2);
            }

         /*   if (gamepad1.right_bumper) {
                shooter.setRightFeederManual(1.0);
            } else {
                shooter.setRightFeederManual(0.0);
            } */
        }
    }


    private double getVisionTurnCorrection(double tx) {
        if (Math.abs(tx) <= VISION_TURN_DEADBAND) {
            return 0.0;
        }

        double turn = -tx * VISION_TURN_KP;

        if (turn > VISION_TURN_MAX) turn = VISION_TURN_MAX;
        if (turn < -VISION_TURN_MAX) turn = -VISION_TURN_MAX;

        return turn;
    }

    public void buildPaths() {
        score1 = follower.pathBuilder()
                .addPath(new BezierLine(currentPose, scorePose))
                .setLinearHeadingInterpolation(currentPose.getHeading(), scorePose.getHeading())
                .build();
    }
}