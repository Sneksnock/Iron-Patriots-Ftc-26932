package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

public class shooter {

    /// ---------- HARDWARE ----------
    public static CRServo Lfeeder, Rfeeder;//lift;
    public static DcMotorEx Lsh, Rsh;
    private static DcMotorEx Lf;
    private static DcMotorEx Rf;
    private static DcMotorEx Lb;
    private static DcMotorEx Rb;
    public static DcMotorEx ie;
    public static Servo ll;
    public static Servo lr;
    public static Servo lift;


    private final ElapsedTime stateTimer = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);

    /// ---------- STATES ----------
    public enum ShootingState {
        IDLE,
        SPIN_UP,
        FEED_ON,
        FEED_OFF
    }

    public enum ShotMode {
        NONE,
        CLOSE,
        FAR,
        CUSTOM
    }

    public static ShootingState shootingState = ShootingState.IDLE;
    public static ShotMode shotMode = ShotMode.NONE;

    /// ---------- FEEDER / INTAKE ----------
    public static double LfeederP = 1.0;
    public static double RfeederP = 1.0;
    public static double ieP = 870;
    public static double off = -1.0;
    public static double offW = 0.0;
    ///--------- KICKSTAND ----------
    public static double upP  = 1.0; // place holder
    /// ---------- BURST TIMING ----------
    public static double FEED_TIME = 575;   // both feeders on
    public static double GAP_TIME = 0;    // pause between bursts

    /// ---------- PIDF ----------
    public static double F = 14;
    public static double P = 400;

    /// ---------- LIGHTS ----------
    public static double red = 0.3;
    public static double green = 0.5;

    /// ---------- SHOOTER ----------
    public static double CLOSE_VELOCITY = 1050;
    public static double FAR_VELOCITY = 1425;
    public static double VELOCITY_TOLERANCE = 20;
    public static double SPIN_TIME = 600;
    public static double SPIN_TIMEFAR = 2000;
    public static double IDLE_VELOCITY = 900;
    public static boolean HOLD_IDLE_SPEED = true;

    /// ---------- TODO: SMART SHOT ----------
    private double customVelocity = CLOSE_VELOCITY;
    private double customSpinTime = SPIN_TIME;

    /// ---------- SHOT COUNT ----------
    /// Now counts BURSTS, not alternating feeder events.
    public int ShotsRemaining = 0;

    public static void init(HardwareMap hwMap) {

        ie = hwMap.get(DcMotorEx.class, "intake");
        Lsh = hwMap.get(DcMotorEx.class, "left_launcher");
        Rsh = hwMap.get(DcMotorEx.class, "right_launcher");
        Lfeeder = hwMap.get(CRServo.class, "left_feeder");
        Rfeeder = hwMap.get(CRServo.class, "right_feeder");
        Lf = hwMap.get(DcMotorEx.class, "left_front_drive");
        Rf = hwMap.get(DcMotorEx.class, "right_front_drive");
        Lb = hwMap.get(DcMotorEx.class, "left_back_drive");
        Rb = hwMap.get(DcMotorEx.class, "right_back_drive");
        ll = hwMap.get(Servo.class, "light_left");
        lr = hwMap.get(Servo.class, "light_right");
       // lift = hwMap.get(CRServo.class, "lift");
        lift = hwMap.get(Servo.class, "lift");
        ie.setDirection(DcMotorSimple.Direction.REVERSE);
        Lf.setDirection(DcMotorSimple.Direction.REVERSE);
        Lb.setDirection(DcMotorSimple.Direction.REVERSE);
        Rsh.setDirection(DcMotorSimple.Direction.REVERSE);
        Rfeeder.setDirection(DcMotorSimple.Direction.REVERSE);

        Lsh.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        Rsh.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);

        PIDFCoefficients pidfCoefficients = new PIDFCoefficients(P, 0, 0, F);
        Lsh.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidfCoefficients);
        Rsh.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidfCoefficients);

        shootingState = ShootingState.IDLE;
        shotMode = ShotMode.NONE;

        ie.setPower(0);
        Lfeeder.setPower(0);
        Rfeeder.setPower(0);
        ll.setPosition(red);
        lr.setPosition(red);
    }

    /// ---------- LIGHTS ----------
    public void updateLights() {
        if (ShotsRemaining > 0 || isBusy()) {
            ll.setPosition(green);
            lr.setPosition(green);
        } else {
            ll.setPosition(red);
            lr.setPosition(red);
        }
    }

    /// ---------- PUBLIC REQUEST METHODS ----------
    public void requestCloseShot() {
        if (!isBusy()) {
            ShotsRemaining = 4;
            shotMode = ShotMode.CLOSE;
            stateTimer.reset();
        }
    }

    public void requestFarShot() {
        if (!isBusy()) {
            ShotsRemaining = 4;
            shotMode = ShotMode.FAR;
            stateTimer.reset();
        }
    }

    /// ---------- TODO: SMART SHOT ----------
    public void requestVelocityShot(double targetVelocity) {
        if (!isBusy()) {
            ShotsRemaining = 4;
            shotMode = ShotMode.CUSTOM;
            customVelocity = targetVelocity;

            if (targetVelocity >= FAR_VELOCITY - 25) {
                customSpinTime = SPIN_TIMEFAR;
            } else {
                customSpinTime = SPIN_TIME;
            }

            stateTimer.reset();
        }
    }
    /// ---------- VISION SHOT SUPPORT ----------
    private boolean useVisionVelocity = false;
    private double visionVelocity = CLOSE_VELOCITY;

    public void setVisionVelocity(double targetVelocity) {
        visionVelocity = targetVelocity;
    }

    public double getVisionVelocity() {
        return visionVelocity;
    }

    public void enableVisionVelocity(boolean enabled) {
        useVisionVelocity = enabled;
    }

    public boolean isUsingVisionVelocity() {
        return useVisionVelocity;
    }

    public void requestVisionShot() {
        if (!isBusy()) {
            ShotsRemaining = 4;
            shotMode = ShotMode.CUSTOM;
            customVelocity = visionVelocity;

            if (customVelocity >= FAR_VELOCITY - 25) {
                customSpinTime = SPIN_TIMEFAR;
            } else {
                customSpinTime = SPIN_TIME;
            }

            stateTimer.reset();
        }
    }

    public void requestVisionShot(int burstCount) {
        if (!isBusy()) {
            ShotsRemaining = burstCount;
            shotMode = ShotMode.CUSTOM;
            customVelocity = visionVelocity;

            if (customVelocity >= FAR_VELOCITY - 25) {
                customSpinTime = SPIN_TIMEFAR;
            } else {
                customSpinTime = SPIN_TIME;
            }

            stateTimer.reset();
        }
    }
    /// ---------- BURST COUNT ----------
    public void requestCloseShot(int burstCount) {
        if (!isBusy()) {
            ShotsRemaining = burstCount;
            shotMode = ShotMode.CLOSE;
            stateTimer.reset();
        }
    }

    public void requestFarShot(int burstCount) {
        if (!isBusy()) {
            ShotsRemaining = burstCount;
            shotMode = ShotMode.FAR;
            stateTimer.reset();
        }
    }

    public void requestVelocityShot(double targetVelocity, int burstCount) {
        if (!isBusy()) {
            ShotsRemaining = burstCount;
            shotMode = ShotMode.CUSTOM;
            customVelocity = targetVelocity;
            customSpinTime = (targetVelocity >= FAR_VELOCITY - 25) ? SPIN_TIMEFAR : SPIN_TIME;
            stateTimer.reset();
        }
    }

    /// ---------- LEGACY COMPATIBILITY ----------
    public void shoot() {
        requestCloseShot();
    }

    public void shootFar() {
        requestFarShot();
    }

    /// ---------- STATE / STATUS ----------
    public ShootingState getState() {
        return shootingState;
    }

    public boolean isBusy() {
        return shootingState != ShootingState.IDLE || shotMode != ShotMode.NONE || ShotsRemaining > 0;
    }
    public void idleSpeed() {
        if (!isBusy()) {
            Lsh.setVelocity(IDLE_VELOCITY);
            Rsh.setVelocity(IDLE_VELOCITY);
        }
    }
    public void cancelShot() {
        Lsh.setPower(0.0);
        Rsh.setPower(0.0);
        Lfeeder.setPower(0.0);
        Rfeeder.setPower(0.0);
        ie.setPower(-1.0);

        ShotsRemaining = 0;
        shootingState = ShootingState.IDLE;
        shotMode = ShotMode.NONE;
        updateLights();
    }

    public double getTargetVelocity() {
        switch (shotMode) {
            case FAR:
                return FAR_VELOCITY;

            case CUSTOM:
                return customVelocity;

            case CLOSE:
            default:
                if (useVisionVelocity) {
                    return visionVelocity;
                }
                return CLOSE_VELOCITY;
        }
    }


    public double getTargetSpinTime() {
        switch (shotMode) {
            case FAR:
                return SPIN_TIMEFAR;
            case CUSTOM:
                return customSpinTime;
            case CLOSE:
            default:
                return SPIN_TIME;
        }
    }

    /// ---------- MANUAL FEEDER HELPERS ----------
    public void setLeftFeederManual(double power) {
        Lfeeder.setPower(power);
    }

    public void setRightFeederManual(double power) {
        Rfeeder.setPower(power);
    }

    public void setBothFeedersManual(double power) {
        Lfeeder.setPower(power);
        Rfeeder.setPower(power);
    }

    /// ---------- UPDATE ----------
    public void update() {

        updateLights();

        double targetVelocity = getTargetVelocity();

        boolean atSpeed =
                Math.abs(Lsh.getVelocity() - targetVelocity) <= VELOCITY_TOLERANCE &&
                        Math.abs(Rsh.getVelocity() - targetVelocity) <= VELOCITY_TOLERANCE;

        switch (shootingState) {

            case IDLE:

                Lfeeder.setPower(off);
                Rfeeder.setPower(off);

                if (shotMode != ShotMode.NONE && ShotsRemaining > 0) {

                    ie.setVelocity(ieP);

                    Lsh.setVelocity(targetVelocity);
                    Rsh.setVelocity(targetVelocity);

                    stateTimer.reset();
                    shootingState = ShootingState.SPIN_UP;
                }
                else {

                    if (HOLD_IDLE_SPEED) {
                        Lsh.setVelocity(IDLE_VELOCITY);
                        Rsh.setVelocity(IDLE_VELOCITY);
                    }
                    else {
                        Lsh.setPower(0.0);
                        Rsh.setPower(0.0);
                    }

                    ie.setPower(0.0);
                }

                break;


            case SPIN_UP:

                if (atSpeed || stateTimer.milliseconds() >= getTargetSpinTime()) {

                    ie.setVelocity(ieP);

                    Lfeeder.setPower(LfeederP);
                    Rfeeder.setPower(RfeederP);

                    stateTimer.reset();
                    shootingState = ShootingState.FEED_ON;
                }

                break;


            case FEED_ON:

                if (stateTimer.milliseconds() >= FEED_TIME) {

                    Lfeeder.setPower(0.0);
                    Rfeeder.setPower(0.0);

                    ShotsRemaining--;

                    stateTimer.reset();

                    if (ShotsRemaining > 0) {
                        shootingState = ShootingState.FEED_OFF;
                    }
                    else {

                        shootingState = ShootingState.IDLE;
                        shotMode = ShotMode.NONE;

                    }

                }

                break;


            case FEED_OFF:

                if (stateTimer.milliseconds() >= GAP_TIME) {

                    Lfeeder.setPower(LfeederP);
                    Rfeeder.setPower(RfeederP);

                    stateTimer.reset();

                    shootingState = ShootingState.FEED_ON;

                }

                break;

        }

    }

    }
