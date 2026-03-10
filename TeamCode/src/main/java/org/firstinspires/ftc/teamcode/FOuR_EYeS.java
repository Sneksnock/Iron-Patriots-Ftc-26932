package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.HardwareMap;

public class FOuR_EYeS {

    /// ---------- TODO: LIMELIGHT HARDWARE ----------
    /// Replace this with the real Limelight class once installed
    /// Example:
    /// private Limelight3A limelight;

    private boolean validTarget = false;
    private double yawErrorDeg = 0;
    private double distanceInches = 0;

    public void init(HardwareMap hardwareMap) {

        /// ---------- TODO: LIMELIGHT INIT ----------
        /// Example later:
        /// limelight = hardwareMap.get(Limelight3A.class, "limelight");
        /// limelight.start();

        validTarget = false;
        yawErrorDeg = 0;
        distanceInches = 0;
    }

    public void update() {

        /// ---------- TODO: LIMELIGHT UPDATE ----------
        /// Read the latest camera frame here

        validTarget = false;
        yawErrorDeg = 0;
        distanceInches = 0;
    }

    public boolean hasValidTarget() {
        return validTarget;
    }

    public double getYawErrorDeg() {
        return yawErrorDeg;
    }

    public double getDistanceInches() {
        return distanceInches;
    }
}