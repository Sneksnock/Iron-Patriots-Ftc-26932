package org.firstinspires.ftc.teamcode;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.pedropathing.util.Timer;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
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
@TeleOp(name = "67", group = "TeleOp")
public class Temp extends LinearOpMode {
    private PanelsTelemetry panelsTelemetry;
    /// ---------------- HARDWARE ----------------
    private Follower follower;
    private DcMotorEx Lf, Rf, Lb, Rb;
    private DcMotorEx Lsh, Rsh;
    private DcMotorEx intake;
    private CRServo Lfeeder, Rfeeder;
    private GoBildaPinpointDriver odo;


    /// ---------------- INIT ----------------
    @Override
    public void runOpMode() {

        /// ---------------- Hardware ----------------
        follower = Constants.createFollower(hardwareMap);

        // Panels Telemetry Initialization

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
        // Note: Rsh and intake directions were set twice in original code, kept as requested
        Rsh.setDirection(DcMotorSimple.Direction.REVERSE);
        intake.setDirection(DcMotorSimple.Direction.REVERSE);

        /// ---------------- odometry ----------------
        odo.setOffsets(-84.0, -168.0, DistanceUnit.MM);
        odo.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        odo.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.FORWARD, GoBildaPinpointDriver.EncoderDirection.FORWARD);
        odo.resetPosAndIMU();

        waitForStart();

        odo.resetPosAndIMU();
        Pose2D startingPositon = new Pose2D(DistanceUnit.INCH, 22.583, 60.082, AngleUnit.RADIANS, 2.566);
        odo.setPosition(startingPositon);

        while (opModeIsActive()) {
            // Update Pedro Pathing and Panels every loop
            follower.update();


            /// ---------------- Privates, and stuff(categorizing name) that isn't hardware----------------
            // Logic moved inside loop so it updates during TeleOp
            Pose currentPose = follower.getPose();
            double x = currentPose.getX();
            double y = currentPose.getY();
            double heading = currentPose.getHeading();


            moveRobot();

            // ---------------- odo telemetry ----------------
            telemetry.addData("status", "Running");
            telemetry.addData("X Position", x);
            telemetry.addData("Y Position", y);
            telemetry.addData("Heading Deg", Math.toDegrees(heading));
            telemetry.update();
        }
    }

    /// ---------------- controls ----------------
    public void moveRobot() {
        double forward = -gamepad1.left_stick_x;
        double strafe = -gamepad1.left_stick_y;
        double rotate = gamepad1.right_stick_x;
        if(gamepad1.optionsWasPressed()){
            odo.resetPosAndIMU();
        }

        //---------------- math ----------------
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

        // ---------------- Panels telemetry ----------------


        //---------------- wheel telemetry ----------------
        telemetry.addData("Forward Speed : ", globalForward);
        telemetry.addData("strafeSpeed :", globalStrafe);
    }
}
