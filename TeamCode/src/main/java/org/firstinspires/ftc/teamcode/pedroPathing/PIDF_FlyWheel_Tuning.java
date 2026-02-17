
package org.firstinspires.ftc.teamcode.pedroPathing;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

@TeleOp
public class PIDF_FlyWheel_Tuning extends OpMode {
    public DcMotorEx Lsh;
    public DcMotorEx Rsh;
    double high = 1110;
    double low = 1100;

    double target = high;
    double F = 0;
    double P = 0;
    double[] stepSizes = {10.0, 1.0, 0.1, 0.001, 0.0001};

    int step = 1;
    @Override
    public void init() {
        Lsh = hardwareMap.get(DcMotorEx.class, "left_launcher");
        Rsh = hardwareMap.get(DcMotorEx.class, "right_launcher");
        Lsh.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        Rsh.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        Rsh.setDirection(DcMotorSimple.Direction.REVERSE);

        PIDFCoefficients pidfCoefficients = new PIDFCoefficients(P, 0, 0, F);
        Lsh.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidfCoefficients);
        Rsh.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidfCoefficients);
        telemetry.addLine(" Init Complete");
    }

    @Override
    public void loop() {
if (gamepad1.yWasPressed()){
    if (target == high) {
        target = low;
    } else { target = high; }

    if (gamepad1.bWasPressed()) {
        step = (step + 1) % stepSizes.length;
    }


    if (gamepad1.dpadLeftWasPressed()) {
    F -= stepSizes[step];
    }

    if (gamepad1.dpadRightWasPressed()) {
        F += stepSizes[step];
    }

    if (gamepad1.dpadUpWasPressed()) {
        P += stepSizes[step];
    }

    if (gamepad1.dpadDownWasPressed()) {
        P -= stepSizes[step];
    }

    PIDFCoefficients pidfCoefficients = new PIDFCoefficients(P, 0, 0, F);
    Lsh.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidfCoefficients);
    Rsh.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidfCoefficients);


    Lsh.setVelocity(target);
    Rsh.setVelocity(target);

    double Lcur = Lsh.getVelocity();
    double Lerror = target - Lcur;
    double Rcur = Rsh.getVelocity();
    double Rerror = target - Rcur;

    telemetry.addData("Target", target);
    telemetry.addData(" Left Current Velocity","%.2f", Lcur);
    telemetry.addData("Right Current Velocity","%.2f", Rcur);
    telemetry.addData("Left Error","%.2f", Lerror);
    telemetry.addData("Right Error","%.2f", Rerror);
    telemetry.addData("Tuning P","%.4f", P);
    telemetry.addData("Tuning F","%.4f", F);
    telemetry.addData("Step Size","%.4f", stepSizes[step]);

}






    }
}

