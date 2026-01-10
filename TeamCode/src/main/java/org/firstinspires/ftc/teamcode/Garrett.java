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
    private static final double CLOSE_VELOCITY =1100;
    private static final double VELOCITY_TOLERANCE = 25;
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
        /*1. move backwards into firing pos */
        moveBackward(30);
        shootAll();
        /*2. Fire two balls using close fire */
    }

    //Move Forward
    public void shootAll() {
        Rsh.setVelocity(CLOSE_VELOCITY);
        Lsh.setVelocity(CLOSE_VELOCITY);
        intake.setPower(0.6);
        boolean atSpeed = false;

        //This loop is constantly asking the robot If it has reached the correct speed, if the robot hasn't then it does nothing.
        while(atSpeed == false) {
            atSpeed = Math.abs(Lsh.getVelocity() - CLOSE_VELOCITY) <= VELOCITY_TOLERANCE &&
                    Math.abs(Rsh.getVelocity() - CLOSE_VELOCITY) <= VELOCITY_TOLERANCE;
            telemetry.addData("Left shooter velocity:", Lsh.getVelocity());
            telemetry.addData("Right shooter velocity:", Rsh.getVelocity());
            telemetry.addData("firing true", atSpeed);
            telemetry.update();
            sleep(100);
        }

        Lfeeder.setPower(1.0);
        sleep(1600);

        //Stop to keep the second ball from coming up
        Lfeeder.setPower(0);

        //Wait for the flywheel to get up to speed
        sleep(500);

        //Shoot the first ball on the left
        Rfeeder.setPower(1.0);
        sleep(2000); //Change if super servo
        Rfeeder.setPower(0);

        sleep(500);

        Lfeeder.setPower(1.0);
        sleep(1600);
        Lfeeder.setPower(0.0);

        sleep(500);

        Rfeeder.setPower(1.0);
        sleep(2000); //Change if super servo
        Rfeeder.setPower(0.0);
    }
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

/*
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
 */


    /* public void Cf() {
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
        sleep(500);
    } */

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



