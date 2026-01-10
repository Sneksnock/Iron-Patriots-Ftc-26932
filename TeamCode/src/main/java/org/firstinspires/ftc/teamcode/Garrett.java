package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

import java.util.Locale;

@Autonomous(name ="Blue Side", group = "Examples")
public class Garrett extends LinearOpMode {
    GoBildaPinpointDriver odo;
    DcMotorEx Lf;
    DcMotorEx Rf;
    DcMotorEx Lb;
    DcMotorEx Rb;
    DcMotorEx Lsh;
    DcMotorEx Rsh;
    DcMotorEx intake;
    CRServo Lfeeder;
    CRServo Rfeeder;
    Servo Di;

    @Override
    public void runOpMode() {
        odo = hardwareMap.get(GoBildaPinpointDriver.class, "odo");
        Lf = hardwareMap.get(DcMotorEx.class, "left_front_drive");
        Rf = hardwareMap.get(DcMotorEx.class, "right_front_drive");
        Lb = hardwareMap.get(DcMotorEx.class, "left_back_drive");
        Lsh = hardwareMap.get(DcMotorEx.class, "left_launcher");
        Rsh = hardwareMap.get(DcMotorEx.class, "right_launcher");
        intake = hardwareMap.get(DcMotorEx.class, "intake");
        Lfeeder = hardwareMap.get(CRServo.class, "left_feeder");
        Rfeeder = hardwareMap.get(CRServo.class, "right_feeder");
        Rb = hardwareMap.get(DcMotorEx.class, "right_back_drive");

        odo.setOffsets(-84.0, -168, DistanceUnit.MM);
        odo.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        odo.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.FORWARD, GoBildaPinpointDriver.EncoderDirection.FORWARD);
        odo.resetPosAndIMU();

        Lf.setDirection(DcMotorSimple.Direction.REVERSE);
        Lb.setDirection(DcMotorSimple.Direction.REVERSE);
        //Acutal Auton // Preloaded shots
        waitForStart();
        /*1. move backwards into firing pos */ moveBackward(30);
        /*2. Fire two balls using close fire */ Cf();
        /* 3. intake third ball*/ intake.setPower(-1);
        sleep(3000);
        /* 4. Fire third ball */ Cf();
        /* 5. rotate 135 degrees */


    }

    //Move Forwards
    public void moveForward(double distance) {
        odo.update();

        double startDistance = odo.getPosX(DistanceUnit.INCH);
        double movedDistance = 0;

        //Set motors to start moving forward
        Lf.setPower(.75);
        Rf.setPower(.75);
        Lb.setPower(.75);
        Rb.setPower(.75);

        while (opModeIsActive() && movedDistance < distance) {
            odo.update();
            movedDistance = odo.getPosX(DistanceUnit.INCH) - startDistance;
        }
        //Brakes
        Lf.setPower(-.15);
        Rf.setPower(-.15);
        Lb.setPower(-.15);
        Rb.setPower(-.15);
        sleep(300);
        Lf.setPower(0);
        Rf.setPower(0);
        Lb.setPower(0);
        Rb.setPower(0);
    }

    //Move Backwards
    public void moveBackward(double distance) {
        odo.update();

        double startDistance = odo.getPosX(DistanceUnit.INCH);
        double movedDistance = 0;

        Lf.setPower(-1);
        Rf.setPower(-1);
        Lb.setPower(-1);
        Rb.setPower(-1);

        while (opModeIsActive() && (-1 * movedDistance) < distance) {
            odo.update();
            movedDistance = odo.getPosX(DistanceUnit.INCH) - startDistance;
        }
        //Brakes
        Lf.setPower(.15);
        Rf.setPower(.15);
        Lb.setPower(.15);
        Rb.setPower(.15);
        sleep(300);
        Lf.setPower(0);
        Rf.setPower(0);
        Lb.setPower(0);
        Rb.setPower(0);
    }

    //Move Left
    public void moveLeft(double distance) {
        odo.update();

        double startDistance = odo.getPosY(DistanceUnit.INCH);
        double movedDistance = 0;

        Lf.setPower(.75);
        Rf.setPower(-.75);
        Lb.setPower(-.75);
        Rb.setPower(.75);

        while (opModeIsActive() && (movedDistance) < distance) {
            odo.update();
            movedDistance = odo.getPosY(DistanceUnit.INCH) - startDistance;
        }
        //Brakes
        Lf.setPower(-.15);
        Rf.setPower(-.15);
        Lb.setPower(.15);
        Rb.setPower(.15);
        sleep(300);
        Lf.setPower(0);
        Rf.setPower(0);
        Lb.setPower(0);
        Rb.setPower(0);
    }

    //Move RIGHT
    public void moveRight(double distance) {
        odo.update();

        double startDistance = odo.getPosY(DistanceUnit.INCH);
        double movedDistance = 0;

        Lf.setPower(-.75);
        Rf.setPower(.75);
        Lb.setPower(.75);
        Rb.setPower(-.75);

        while (opModeIsActive() && (-1 * movedDistance) < distance) {
            odo.update();
            movedDistance = odo.getPosY(DistanceUnit.INCH) - startDistance;
        }
        //Brakes
        Lf.setPower(.15);
        Rf.setPower(-.15);
        Lb.setPower(-.15);
        Rb.setPower(.15);
        sleep(300);
        Lf.setPower(0);
        Rf.setPower(0);
        Lb.setPower(0);
        Rb.setPower(0);
    }

    public void Ff() {
        Lsh.setPower(1.5);
        Rsh.setPower(-1.5);
        sleep(250);
        Lfeeder.setPower(1);
        Rfeeder.setPower(-1);
        sleep(3000);
        //`
        Lsh.setPower(.25);
        Rsh.setPower(-.25);
        Lfeeder.setPower(-1);
        Rfeeder.setPower(1);
        sleep(500);
        Lsh.setPower(0);
        Rsh.setPower(0);
        Lfeeder.setPower(0);
        Rfeeder.setPower(0);
    }

    public void Cf() {
        Lsh.setPower(.54);
        Rsh.setPower(-.54);
        sleep(1650);
        Lfeeder.setPower(1);
        sleep(500);
        Rfeeder.setPower(-1);
        //`
        /* Lsh.setPower(-.10);
        Rsh.setVelocity(-.1);
        Lfeeder.setPower(.1);
        Rfeeder.setPower(.1);
        sleep(500);*/
    }

    //rotate clockwise
    public void RotateClockwise(double distance) {
        odo.update();

        double startDistance = odo.getPosX(DistanceUnit.INCH);
        double movedDistance = 0;
        double movedDegrees = 0;

        Lf.setPower(-.75);
        Rf.setPower(.75);
        Lb.setPower(-.75);
        Rb.setPower(.75);

        while (opModeIsActive() && (movedDegrees) < distance) {
            odo.update();
            movedDistance = odo.getPosX(DistanceUnit.INCH) - startDistance;
            movedDegrees = movedDistance / (2 * Math.PI * 6.456) * 360;
            telemetry.addData("MovedDistance", movedDistance);
            telemetry.addData("MovedDeg", movedDegrees);
            telemetry.update();
        }
        //Brakes
        Lf.setPower(.15);
        Rf.setPower(-.15);
        Lb.setPower(.15);
        Rb.setPower(-.15);
        sleep(300);
        Lf.setPower(0);
        Rf.setPower(0);
        Lb.setPower(0);
        Rb.setPower(0);
    }
}



