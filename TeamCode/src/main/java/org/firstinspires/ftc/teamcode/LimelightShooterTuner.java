package org.firstinspires.ftc.teamcode;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@Configurable
@TeleOp(name = "Limelight Shooter Tuner", group = "Tuning")
public class LimelightShooterTuner extends LinearOpMode {

    /// ---------- LIMELIGHT ----------
    public static int PIPELINE = 1;

    /// ---------- SHOOTER TUNING ----------
    // Change this live in Panels
    public static double TARGET_VELOCITY = 1110;

    /// ---------- MANUAL TEST CONTROLS ----------
    public static boolean SPIN_SHOOTER = false;
    public static boolean FEED_BOTH = true;
    public static double FEEDER_POWER = 1.0;

    private Limelight3A limelight;
    private shooter shooter;

    @Override
    public void runOpMode() {

        shooter = new shooter();
        shooter.init(hardwareMap);

        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.stop();
        limelight.pipelineSwitch(PIPELINE);

        telemetry.addLine("Limelight Shooter Tuner Ready");
        telemetry.addLine("Use Panels to change TARGET_VELOCITY");
        telemetry.addLine("Set SPIN_SHOOTER = true to spin launchers");
        telemetry.addLine("Set FEED_BOTH = true to feed rings");
        telemetry.update();

        waitForStart();

        limelight.start();

        while (opModeIsActive()) {
            limelight.pipelineSwitch(PIPELINE);

            LLResult result = limelight.getLatestResult();

            boolean hasTarget = false;
            double tx = 0.0;
            double ty = 0.0;
            double ta = 0.0;
            long staleness = -1;

            if (result != null) {
                staleness = result.getStaleness();

                if (result.isValid()) {
                    hasTarget = true;
                    tx = result.getTx();
                    ty = result.getTy();
                    ta = result.getTa();
                }
            }

            /// ---------- SHOOTER CONTROL ----------
            if (SPIN_SHOOTER) {
                shooter.Lsh.setVelocity(TARGET_VELOCITY);
                shooter.Rsh.setVelocity(TARGET_VELOCITY);
            } else {
                if (shooter.HOLD_IDLE_SPEED) {
                    shooter.Lsh.setVelocity(shooter.IDLE_VELOCITY);
                    shooter.Rsh.setVelocity(shooter.IDLE_VELOCITY);
                } else {
                    shooter.Lsh.setPower(0.0);
                    shooter.Rsh.setPower(0.0);
                }
            }

            /// ---------- FEEDER CONTROL ----------
            if (FEED_BOTH) {
                shooter.Lfeeder.setPower(-FEEDER_POWER);
                shooter.Rfeeder.setPower(-FEEDER_POWER);
                shooter.ie.setVelocity(875);
            } else {
                shooter.Lfeeder.setPower(shooter.off);
                shooter.Rfeeder.setPower(shooter.off);
                shooter.ie.setVelocity(0);
            }

            /// ---------- TELEMETRY ----------
            telemetry.addLine("=== LIMELIGHT ===");
            telemetry.addData("Target Visible", hasTarget);
            telemetry.addData("tx", "%.3f", tx);
            telemetry.addData("ty", "%.3f", ty);
            telemetry.addData("ta", "%.6f", ta);
            telemetry.addData("Staleness (ms)", staleness);

            telemetry.addLine("=== SHOOTER ===");
            telemetry.addData("Target Velocity", TARGET_VELOCITY);
            telemetry.addData("Spin Shooter", SPIN_SHOOTER);
            telemetry.addData("Feed Both", FEED_BOTH);
            telemetry.addData("Left Velocity", "%.2f", shooter.Lsh.getVelocity());
            telemetry.addData("Right Velocity", "%.2f", shooter.Rsh.getVelocity());

            telemetry.addLine("=== CAPTURE ===");
            telemetry.addLine("When the shot works, record:");
            telemetry.addData("POINT", "(%.6f, %.2f)", ta, TARGET_VELOCITY);

            telemetry.update();
        }

        limelight.stop();
        shooter.Lsh.setPower(0.0);
        shooter.Rsh.setPower(0.0);
        shooter.Lfeeder.setPower(shooter.off);
        shooter.Rfeeder.setPower(shooter.off);
        shooter.ie.setPower(0.0);
    }
}