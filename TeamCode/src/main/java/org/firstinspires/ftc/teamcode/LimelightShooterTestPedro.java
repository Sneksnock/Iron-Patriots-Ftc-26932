package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.LimelightShooterMechanism;

@TeleOp(name = "Limelight Shooter Test Pedro", group = "Test")
public class LimelightShooterTestPedro extends LinearOpMode {

    private LimelightShooterMechanism limelightShooter;

    @Override
    public void runOpMode() throws InterruptedException {

        limelightShooter = new LimelightShooterMechanism(
                hardwareMap,
                telemetry,
                LimelightShooterMechanism.RED_PIPELINE_INDEX
        );

        telemetry.addLine("Initialized");
        telemetry.update();

        waitForStart();

        limelightShooter.start();

        while (opModeIsActive()) {

            // Your Pedro drive update would also go in this loop
            // follower.update();



            telemetry.update();
        }

        limelightShooter.stop();
    }
}