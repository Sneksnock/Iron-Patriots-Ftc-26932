package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

// NOTE: We are using the GoBildaPinpointDriver defined in this same package,
// from your example SensorGoBildaPinpointExample. No external import needed.

@TeleOp(name = "Patriots TeleOp", group = "TeleOp")
public class PatriotsTeleop extends LinearOpMode {

    // ---------- DRIVE MOTORS ----------
    private DcMotor lf, rf, lb, rb;

    // ---------- MECHANISMS ----------
    private DcMotor leftLauncher, rightLauncher, intake;
    private CRServo leftFeeder, rightFeeder;
    private Servo diverter;

    // ---------- PINPOINT / ODOMETRY ----------
    private GoBildaPinpointDriver odo;

    // ---------- STATE VARIABLES ----------
    private boolean slowMode = false;
    private boolean lastSlowPress = false;

    private boolean intakeOn = false;
    private boolean lastIntakePress = false;

    private enum FireState { IDLE, SPINUP, FEED }
    private FireState fireState = FireState.IDLE;
    private ElapsedTime fireTimer = new ElapsedTime();

    private static final double SLOW_MULT = 0.4;
    private static final double INTAKE_PWR = 1.0;

    // Launcher timing
    private static final double SPINUP_TIME = 4.5;
    private static final double FEED_TIME   = 2.5;

    // Power levels
    private static final double CLOSE_PWR = 0.65;
    private static final double FAR_PWR   = 0.85;

    // Diverter positions
    private static final double DIV_CLOSE = 0.20;
    private static final double DIV_FAR   = 0.52;

    private double launcherPower = 0;

    @Override
    public void runOpMode() throws InterruptedException {

        // ---------- HARDWARE MAP ----------
        lf = hardwareMap.get(DcMotor.class, "left_front_drive");
        rf = hardwareMap.get(DcMotor.class, "right_front_drive");
        lb = hardwareMap.get(DcMotor.class, "left_back_drive");
        rb = hardwareMap.get(DcMotor.class, "right_back_drive");

        leftLauncher  = hardwareMap.get(DcMotor.class, "left_launcher");
        rightLauncher = hardwareMap.get(DcMotor.class, "right_launcher");
        intake        = hardwareMap.get(DcMotor.class, "intake");

        leftFeeder  = hardwareMap.get(CRServo.class, "left_feeder");
        rightFeeder = hardwareMap.get(CRServo.class, "right_feeder");
        diverter    = hardwareMap.get(Servo.class, "diverter");

        // Use the SAME naming and driver as your example:
        // odo in config, GoBildaPinpointDriver class in this package.
        odo = hardwareMap.get(GoBildaPinpointDriver.class, "odo");

        // ---------- MOTOR DIRECTIONS ----------
        lf.setDirection(DcMotorSimple.Direction.REVERSE);
        lb.setDirection(DcMotorSimple.Direction.REVERSE);
        rf.setDirection(DcMotorSimple.Direction.FORWARD);
        rb.setDirection(DcMotorSimple.Direction.FORWARD);

        leftLauncher.setDirection(DcMotorSimple.Direction.FORWARD);
        rightLauncher.setDirection(DcMotorSimple.Direction.REVERSE);
        intake.setDirection(DcMotorSimple.Direction.REVERSE);

        leftFeeder.setDirection(CRServo.Direction.FORWARD);
        rightFeeder.setDirection(CRServo.Direction.REVERSE);

        // ---------- ZERO POWER ----------
        lf.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rf.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        lb.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rb.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // ---------- ODOMETRY / PINPOINT SETUP ----------
        // These calls are copied directly from your example opmode.
        odo.setOffsets(-84.0, -168.0, DistanceUnit.MM); // tuned for your pods
        odo.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        odo.setEncoderDirections(
                GoBildaPinpointDriver.EncoderDirection.FORWARD,
                GoBildaPinpointDriver.EncoderDirection.FORWARD
        );

        // Reset position + IMU fusion before start
        odo.resetPosAndIMU();

        telemetry.addLine("Patriots TeleOp READY (odo + field-centric)");
        telemetry.addData("X offset", odo.getXOffset(DistanceUnit.MM));
        telemetry.addData("Y offset", odo.getYOffset(DistanceUnit.MM));
        telemetry.addData("Device Version", odo.getDeviceVersion());
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {

            // ---------- UPDATE ODOMETRY ----------
            odo.update();
            Pose2D pose = odo.getPosition();
            double heading = pose.getHeading(AngleUnit.RADIANS);

            // ---------- INPUT (Ansh Gupta style) ----------
            double y  = -gamepad1.left_stick_y;  // forward/back
            double x  =  gamepad1.left_stick_x;  // strafe left/right
            double rx =  gamepad1.right_stick_x; // rotation

            // ---------- FIELD-CENTRIC TRANSFORM ----------
            // Rotate joystick vector by -heading to get robot-oriented motion in field coordinates.
            double rotX = -(x * Math.cos(-heading) - y * Math.sin(-heading));
            double rotY = -(x * Math.sin(-heading) + y * Math.cos(-heading));

            // ---------- MECANUM MIX ----------
            double lfP = rotY + rotX + rx;
            double rfP = rotY - rotX - rx;
            double lbP = rotY - rotX + rx;
            double rbP = rotY + rotX - rx;

            // ---------- NORMALIZE ----------
            double max = Math.max(1.0,
                    Math.max(Math.abs(lfP),
                            Math.max(Math.abs(rfP),
                                    Math.max(Math.abs(lbP), Math.abs(rbP)))));

            lfP /= max;
            rfP /= max;
            lbP /= max;
            rbP /= max;

            // ---------- SLOW MODE ----------
            if (gamepad1.right_stick_button && !lastSlowPress) {
                slowMode = !slowMode;
            }
            lastSlowPress = gamepad1.right_stick_button;

            double mult = slowMode ? SLOW_MULT : 1.0;

            lf.setPower(lfP * mult);
            rf.setPower(rfP * mult);
            lb.setPower(lbP * mult);
            rb.setPower(rbP * mult);

            // ---------- INTAKE TOGGLE ----------
            if (gamepad1.left_stick_button && !lastIntakePress) {
                intakeOn = !intakeOn;
            }
            lastIntakePress = gamepad1.left_stick_button;

            intake.setPower(intakeOn ? INTAKE_PWR : 0);

            // ---------- DIVERTER ----------
            if (gamepad1.left_bumper)  diverter.setPosition(DIV_CLOSE);
            if (gamepad1.right_bumper) diverter.setPosition(DIV_FAR);

            // ---------- FIRE SEQUENCES ----------
            if (gamepad1.dpad_left && fireState == FireState.IDLE)
                startFire(CLOSE_PWR);

            if (gamepad1.dpad_right && fireState == FireState.IDLE)
                startFire(FAR_PWR);

            if (gamepad1.dpad_down)
                stopFire();

            handleFire();

            // ---------- TELEMETRY ----------
            telemetry.addData("X (mm)", pose.getX(DistanceUnit.MM));
            telemetry.addData("Y (mm)", pose.getY(DistanceUnit.MM));
            telemetry.addData("Heading°", Math.toDegrees(heading));
            telemetry.addData("Slow Mode", slowMode);
            telemetry.addData("Intake", intakeOn);
            telemetry.addData("Fire State", fireState);
            telemetry.addData("Device Status", odo.getDeviceStatus());
            telemetry.update();
        }
    }

    // ================= FIRE LOGIC =================

    private void startFire(double pwr) {
        launcherPower = pwr;
        leftLauncher.setPower(pwr);
        rightLauncher.setPower(pwr);
        leftFeeder.setPower(0);
        rightFeeder.setPower(0);
        fireTimer.reset();
        fireState = FireState.SPINUP;
    }

    private void handleFire() {
        switch (fireState) {

            case SPINUP:
                if (fireTimer.seconds() >= SPINUP_TIME) {
                    leftFeeder.setPower(1.0);
                    rightFeeder.setPower(1.0);
                    fireTimer.reset();
                    fireState = FireState.FEED;
                }
                break;

            case FEED:
                if (fireTimer.seconds() >= FEED_TIME) {
                    stopFire();
                }
                break;

            case IDLE:
            default:
                // do nothing
                break;
        }
    }

    private void stopFire() {
        leftLauncher.setPower(0);
        rightLauncher.setPower(0);
        leftFeeder.setPower(0);
        rightFeeder.setPower(0);
        fireState = FireState.IDLE;
    }
}
