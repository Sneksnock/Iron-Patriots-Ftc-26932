package      org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@TeleOp(name = "FieldCentricTeleOp", group = "TeleOp")
public class TeleOpDecode2026 extends LinearOpMode {

    private DcMotorEx left_front_drive, right_front_drive, left_back_drive, right_back_drive;
    private DcMotorEx left_launcher, right_launcher, intake;
    private CRServo left_feeder, right_feeder;
    private Servo diverter;
    private GoBildaPinpointDriver odo;

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
    // ---------------- SHOOTER CONSTANTS ----------------
    private static final double CLOSE_VELOCITY =1100;
    private static final double VELOCITY_TOLERANCE = 25;

    @Override
    public void runOpMode() {
        left_front_drive = hardwareMap.get(DcMotorEx.class, "left_front_drive");
        right_front_drive = hardwareMap.get(DcMotorEx.class, "right_front_drive");
        left_back_drive = hardwareMap.get(DcMotorEx.class, "left_back_drive");
        right_back_drive = hardwareMap.get(DcMotorEx.class, "right_back_drive");

        left_launcher = hardwareMap.get(DcMotorEx.class, "left_launcher");
        right_launcher = hardwareMap.get(DcMotorEx.class, "right_launcher");
        intake = hardwareMap.get(DcMotorEx.class, "intake");

        left_feeder = hardwareMap.get(CRServo.class, "left_feeder");
        right_feeder = hardwareMap.get(CRServo.class, "right_feeder");
        diverter = hardwareMap.get(Servo.class, "diverter");

        odo = hardwareMap.get(GoBildaPinpointDriver.class, "odo");

        right_front_drive.setDirection(DcMotorSimple.Direction.REVERSE);
        right_back_drive.setDirection(DcMotorSimple.Direction.REVERSE);
        right_launcher.setDirection(DcMotorSimple.Direction.REVERSE);
        intake.setDirection(DcMotorSimple.Direction.REVERSE);


        //odo
        odo.setOffsets(-84.0, -168.0, DistanceUnit.MM);
        odo.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        odo.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.FORWARD, GoBildaPinpointDriver.EncoderDirection.FORWARD);
        odo.resetPosAndIMU();
        waitForStart();

        odo.resetPosAndIMU();

        while (opModeIsActive()) {
            // Reset heading
            if (gamepad1.options) {
                odo.resetPosAndIMU();
            }

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
                left_feeder.setPower(1.0);
            } else if (gamepad1.left_bumper){
                left_feeder.setPower(0.0);
            }
            if (gamepad1.right_bumper){
                right_feeder.setPower(1.0);
            } else if (gamepad1.right_bumper){
                right_feeder.setPower(0.0);
            }

            // Firing sequence (triggers)
            if (!firing && gamepad1.left_trigger > 0.2) {
                left_launcher.setVelocity(1150);
                right_launcher.setVelocity(1150);
                firing = true;
                firingStartTime = System.currentTimeMillis();
            } else if (!firing && gamepad1.right_trigger > 0.2) {
                left_launcher.setPower(.85);
                right_launcher.setPower(0.85);
                firing = true;
                firingStartTime = System.currentTimeMillis();
            } else if (firing && (System.currentTimeMillis() - firingStartTime >=1500 )) {
                // Start feeder after 1s
                left_feeder.setPower(1.0);
                right_feeder.setPower(-1.0);
            }

            if (firing && (System.currentTimeMillis() - firingStartTime >= 3500)) {
                left_feeder.setPower(0.0);
                right_feeder.setPower(0.0);
                left_launcher.setPower(0.0);
                right_launcher.setPower((0.0));
                firing = false;
            }


            // Emergency stop
            if (gamepad1.dpad_left) {
                left_launcher.setPower(0.0);
                right_launcher.setPower(0.0);
                left_feeder.setPower(0.0);
                right_feeder.setPower(0.0);
                firing = false;
            }


            double y = gamepad1.left_stick_y;
            double x = gamepad1.left_stick_x;
            double rx = -gamepad1.right_stick_x;

            double heading = odo.getHeading(AngleUnit.DEGREES);
            telemetry.addData("Heading",heading);
            telemetry.update();

            //double rotX = -x * Math.cos(-botHeading) - y * Math.sin(-botHeading);
            //double rotY = -x * Math.sin(-botHeading) + y * Math.cos(-botHeading);

            double rotX = -(x * Math.cos(-heading) - y * Math.sin(-heading));
            double rotY = -(x * Math.sin(-heading) + y * Math.cos(-heading));

            double denominator = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(rx), 1.0);
            double frontLeftPower = (rotY + rotX + rx) / denominator;
            double backLeftPower = (rotY - rotX + rx) / denominator;
            double frontRightPower = (rotY - rotX - rx) / denominator;
            double backRightPower = (rotY + rotX - rx) / denominator;

            double speedMultiplier = slowMode ? 0.4 : 1.0;

            left_front_drive.setPower(frontLeftPower * speedMultiplier);
            left_back_drive.setPower(backLeftPower * speedMultiplier);
            right_front_drive.setPower(frontRightPower * speedMultiplier);
            right_back_drive.setPower(backRightPower * speedMultiplier);
        }
    }
}
