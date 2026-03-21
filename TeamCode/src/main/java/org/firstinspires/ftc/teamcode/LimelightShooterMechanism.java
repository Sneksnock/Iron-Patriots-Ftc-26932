package org.firstinspires.ftc.teamcode;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.util.List;

@Configurable
public class LimelightShooterMechanism {

    // ---------- PIPELINES ----------
    public static int RED_PIPELINE_INDEX = 1;
    public static int BLUE_PIPELINE_INDEX = 0;

    // ---------- ALLIANCE ----------
    public static boolean RED_ALLIANCE = true;

    // ---------- GOAL TAG FILTER ----------
    public static boolean USE_TAG_FILTER = false;

    public static int RED_GOAL_TAG_LEFT = 5;
    public static int RED_GOAL_TAG_RIGHT = 6;

    public static int BLUE_GOAL_TAG_LEFT = 1;
    public static int BLUE_GOAL_TAG_RIGHT = 2;

    // ---------- LIVE-TUNABLE SHOOTER VALUES ----------
    public static double TARGET_VELOCITY = 1110;
    public static double VELOCITY_STEP = 25;

    // velocity = TA_SLOPE * ta + TA_INTERCEPT
    public static boolean USE_TA_TO_VELOCITY = true;
    public static double TA_SLOPE = -146.38023;
    public static double TA_INTERCEPT = 1326.457;

    // ---------- PIDF TUNING ----------
    public static double SHOOTER_P = 400;
    public static double SHOOTER_F = 14;

    // ---------- HEADING AIM ----------
    public static double TURN_KP = 0.03;
    public static double TURN_DEADBAND = 0.25;
    public static double TURN_MAX = 0.45;
    public static double AIM_TOLERANCE_DEG = 0.75;

    private final Limelight3A limelight;
    private final Telemetry telemetry;

    private LLResult finalLLResult = null;

    private boolean lastUp = false;
    private boolean lastDown = false;

    private int selectedTagId = -1;
    private double selectedTagDistance = Double.POSITIVE_INFINITY;
    private int visibleGoalTagCount = 0;

    private int currentPipeline = -1;

    public LimelightShooterMechanism(HardwareMap hardwareMap, Telemetry telemetry, int pipelineIndex) {
        this.telemetry = telemetry;

        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.stop();
        setPipeline(pipelineIndex);

        shooter.Lsh.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        shooter.Rsh.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        applyPidf();
    }

    public void start() {
        finalLLResult = null;
        selectedTagId = -1;
        selectedTagDistance = Double.POSITIVE_INFINITY;
        visibleGoalTagCount = 0;
        limelight.start();
    }

    public void stop() {
        limelight.stop();
        finalLLResult = null;
        selectedTagId = -1;
        selectedTagDistance = Double.POSITIVE_INFINITY;
        visibleGoalTagCount = 0;
    }

    public void setPipeline(int pipelineIndex) {
        if (currentPipeline != pipelineIndex) {
            limelight.pipelineSwitch(pipelineIndex);
            currentPipeline = pipelineIndex;
        }
    }

    public void updateAlliancePipeline() {
        setPipeline(RED_ALLIANCE ? RED_PIPELINE_INDEX : BLUE_PIPELINE_INDEX);
    }

    public LLResult getLimelightResult() {
        return finalLLResult;
    }

    public boolean hasValidTarget() {
        return finalLLResult != null && finalLLResult.isValid() && selectedTagId != -1;
    }

    public int getSelectedTagId() {
        return selectedTagId;
    }

    public double getSelectedTagDistance() {
        return selectedTagDistance;
    }

    public int getVisibleGoalTagCount() {
        return visibleGoalTagCount;
    }

    public double getTa() {
        if (finalLLResult != null && finalLLResult.isValid()) {
            return finalLLResult.getTa();
        }
        return 0.0;
    }

    public double getTx() {
        if (finalLLResult != null && finalLLResult.isValid()) {
            return finalLLResult.getTx();
        }
        return 0.0;
    }

    public double getTy() {
        if (finalLLResult != null && finalLLResult.isValid()) {
            return finalLLResult.getTy();
        }
        return 0.0;
    }

    public double getSuggestedVelocity() {
        if (USE_TA_TO_VELOCITY && hasValidTarget()) {
            double suggested = (TA_SLOPE * finalLLResult.getTa()) + TA_INTERCEPT;
            return Math.max(0.0, suggested);
        }
        return TARGET_VELOCITY;
    }

    public double getHeadingCorrection() {
        if (!hasValidTarget()) {
            return 0.0;
        }

        double tx = finalLLResult.getTx();

        if (Math.abs(tx) <= TURN_DEADBAND) {
            return 0.0;
        }

        double turn = -tx * TURN_KP;

        if (turn > TURN_MAX) turn = TURN_MAX;
        if (turn < -TURN_MAX) turn = -TURN_MAX;

        return turn;
    }

    public boolean isAimed() {
        return hasValidTarget() && Math.abs(getTx()) <= AIM_TOLERANCE_DEG;
    }

    public void applyPidf() {
        PIDFCoefficients pidf = new PIDFCoefficients(SHOOTER_P, 0, 0, SHOOTER_F);
        shooter.Lsh.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidf);
        shooter.Rsh.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidf);
    }

    public void update(boolean dpadUp, boolean dpadDown) {
        applyPidf();
        updateAlliancePipeline();

        LLResult llResult = limelight.getLatestResult();

        selectedTagId = -1;
        selectedTagDistance = Double.POSITIVE_INFINITY;
        visibleGoalTagCount = 0;
        finalLLResult = null;

        boolean hasTarget = false;
        double tx = 0.0;
        double ty = 0.0;
        double ta = 0.0;
        long staleness = -1;

        if (llResult != null) {
            staleness = llResult.getStaleness();

            if (llResult.isValid()) {
                List<LLResultTypes.FiducialResult> fiducials = llResult.getFiducialResults();

                if (fiducials != null && !fiducials.isEmpty()) {
                    for (LLResultTypes.FiducialResult fiducial : fiducials) {
                        int id = fiducial.getFiducialId();

                        if (!isGoalTag(id)) {
                            continue;
                        }

                        visibleGoalTagCount++;

                        double x = fiducial.getRobotPoseTargetSpace().getPosition().x;
                        double y = fiducial.getRobotPoseTargetSpace().getPosition().y;
                        double z = fiducial.getRobotPoseTargetSpace().getPosition().z;
                        double distance = Math.sqrt((x * x) + (y * y) + (z * z));

                        if (distance < selectedTagDistance) {
                            selectedTagDistance = distance;
                            selectedTagId = id;
                        }
                    }
                }

                boolean acceptResult = !USE_TAG_FILTER || selectedTagId != -1;

                if (acceptResult) {
                    finalLLResult = llResult;
                    hasTarget = true;
                    tx = llResult.getTx();
                    ty = llResult.getTy();
                    ta = llResult.getTa();
                }
            }
        }

        if (dpadUp && !lastUp) {
            TARGET_VELOCITY += VELOCITY_STEP;
        }

        if (dpadDown && !lastDown) {
            TARGET_VELOCITY -= VELOCITY_STEP;
            if (TARGET_VELOCITY < 0) TARGET_VELOCITY = 0;
        }

        lastUp = dpadUp;
        lastDown = dpadDown;

        telemetry.addLine("=== LIMELIGHT ===");
        telemetry.addData("Target Visible", hasTarget);
        telemetry.addData("tx", "%.2f", tx);
        telemetry.addData("ty", "%.2f", ty);
        telemetry.addData("ta", "%.4f", ta);
        telemetry.addData("Staleness (ms)", staleness);

        telemetry.addLine("=== GOAL TAGS ===");
        telemetry.addData("Use Tag Filter", USE_TAG_FILTER);
        telemetry.addData("Alliance", RED_ALLIANCE ? "RED" : "BLUE");
        telemetry.addData("Visible Goal Tags", visibleGoalTagCount);
        telemetry.addData("Selected Goal Tag", selectedTagId);
        telemetry.addData("Selected Tag Dist", selectedTagId != -1 ? "%.3f" : "N/A", selectedTagDistance);

        telemetry.addLine("=== VISION TUNING ===");
        telemetry.addData("Use TA->Velocity", USE_TA_TO_VELOCITY);
        telemetry.addData("Target Velocity", TARGET_VELOCITY);
        telemetry.addData("Suggested Velocity", getSuggestedVelocity());
        telemetry.addData("Heading Correction", getHeadingCorrection());
        telemetry.addData("Aimed", isAimed());
        telemetry.addData("Dpad Up/Down", "Velocity +/-");
    }

    private boolean isGoalTag(int id) {
        if (!USE_TAG_FILTER) {
            return true;
        }

        if (RED_ALLIANCE) {
            return id == RED_GOAL_TAG_LEFT || id == RED_GOAL_TAG_RIGHT;
        } else {
            return id == BLUE_GOAL_TAG_LEFT || id == BLUE_GOAL_TAG_RIGHT;
        }
    }
}