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
    public static CRServo Lfeeder, Rfeeder;
    public static DcMotorEx Lsh, Rsh;
    private static DcMotorEx Lf;
    private static DcMotorEx Rf;
    private static DcMotorEx Lb;
    private static DcMotorEx Rb;
    public static DcMotorEx ie;
    public static Servo ll;
    public static Servo lr;

    private final ElapsedTime stateTimer = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);

    /// ---------- STATES ----------
    public enum ShootingState {
        IDLE,
        SPIN_UP,
        LAUNCH1,
        LAUNCH2,
        LAUNCH3,
        LAUNCH4
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
    public static double LfeederT = 500;
    public static double RfeederT = 500;
    public static double ieP = 0.80;
    public static double off = -0.2;
    public static double offW = 0.0;

    /// ---------- PIDF ----------
    public static double F = 14;
    public static double P = 400;

    /// ---------- LIGHTS ----------
    public static double red = 0.3;
    public static double green = 0.5;

    /// ---------- SHOOTER ----------
    public static double CLOSE_VELOCITY = 1110;
    public static double FAR_VELOCITY = 1425;
    public static double VELOCITY_TOLERANCE = 20;
    public static double SPIN_TIME = 600;
    public static double SPIN_TIMEFAR = 2000;

    /// ---------- TODO: SMART SHOT ----------
    /// This gets used later for Limelight / distance-based shooting
    private double customVelocity = CLOSE_VELOCITY;
    private double customSpinTime = SPIN_TIME;

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
        Lfeeder.setPower(off);
        Rfeeder.setPower(off);
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
    /// use this later when Limelight / distance mapping is ready
    public void requestVelocityShot(double targetVelocity) {
        if (!isBusy()) {
            ShotsRemaining = 4;
            shotMode = ShotMode.CUSTOM;
            customVelocity = targetVelocity;

            /// simple default rule for now
            if (targetVelocity >= FAR_VELOCITY - 25) {
                customSpinTime = SPIN_TIMEFAR;
            } else {
                customSpinTime = SPIN_TIME;
            }

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

    public void cancelShot() {
        Lsh.setPower(0.0);
        Rsh.setPower(0.0);
        Lfeeder.setPower(0.0);
        Rfeeder.setPower(0.0);
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
                    ie.setPower(ieP);
                    Lsh.setVelocity(targetVelocity);
                    Rsh.setVelocity(targetVelocity);

                    stateTimer.reset();
                    shootingState = ShootingState.SPIN_UP;
                }
                break;

            case SPIN_UP:
                if (atSpeed || stateTimer.milliseconds() >= getTargetSpinTime()) {
                    ie.setPower(ieP);
                    Lfeeder.setPower(LfeederP);
                    stateTimer.reset();
                    shootingState = ShootingState.LAUNCH1;
                }
                break;

            case LAUNCH1:
                if (stateTimer.milliseconds() >= LfeederT) {
                    Lfeeder.setPower(0.0);
                    Rfeeder.setPower(RfeederP);
                    stateTimer.reset();
                    ShotsRemaining--;
                    shootingState = ShootingState.LAUNCH2;
                }
                break;

            case LAUNCH2:
                if (stateTimer.milliseconds() >= RfeederT) {
                    Rfeeder.setPower(0.0);
                    Lfeeder.setPower(LfeederP);
                    stateTimer.reset();
                    ShotsRemaining--;
                    shootingState = ShootingState.LAUNCH3;
                }
                break;

            case LAUNCH3:
                if (stateTimer.milliseconds() >= LfeederT) {
                    Lfeeder.setPower(0.0);
                    Rfeeder.setPower(RfeederP);
                    stateTimer.reset();
                    ShotsRemaining--;
                    shootingState = ShootingState.LAUNCH4;
                }
                break;

            case LAUNCH4:
                if (stateTimer.milliseconds() >= RfeederT) {
                    Rfeeder.setPower(0.0);
                    stateTimer.reset();
                    ShotsRemaining--;

                    /// return to idle cleanly
                    shootingState = ShootingState.IDLE;
                    shotMode = ShotMode.NONE;
                }
                break;
        }
    }
}