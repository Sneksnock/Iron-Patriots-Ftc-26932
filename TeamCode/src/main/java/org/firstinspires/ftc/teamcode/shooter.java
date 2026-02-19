package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.util.ElapsedTime;

public class shooter {
    public static CRServo Lfeeder, Rfeeder;
    public static DcMotorEx Lsh, Rsh;
    private static DcMotorEx Lf;
    private static DcMotorEx Rf;
    private static DcMotorEx Lb;
    private static DcMotorEx Rb;
    public static DcMotorEx ie;
    public  ElapsedTime stateTimer = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);

    public enum ShootingState {
        IDLE2,
        IDLE,
        SPIN_UP,
        LAUNCH1,
        LAUNCH2,
        LAUNCH3,
        LAUNCH4,

    }

    public static ShootingState shootingState = ShootingState.IDLE;

    // FEEDER / INTAKE
    public static double LfeederP = 1.0;
    public static double RfeederP = 1.0;
    public static double LfeederT = 900;
    public static double RfeederT = 900;
    public static double ieP = .80;
    public static double off = -0.2;
    public static double offW = 0.0;
   public static double F = 14;
   public static double P = 400;

    // SHOOTER
    public static double CLOSE_VELOCITY = 1110;
    public static double VELOCITY_TOLERANCE = 20;
    public static double SPIN_TIME = 750;


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

        ie.setDirection(DcMotorSimple.Direction.REVERSE);
        Lf.setDirection(DcMotorSimple.Direction.REVERSE);
        Lb.setDirection(DcMotorSimple.Direction.REVERSE);
        Rsh.setDirection(DcMotorSimple.Direction.REVERSE);
        Lsh.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        Rsh.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        Rfeeder.setDirection(DcMotorSimple.Direction.REVERSE);

        PIDFCoefficients pidfCoefficients = new PIDFCoefficients(P, 0, 0, F);
        Lsh.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidfCoefficients);
        Rsh.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidfCoefficients);

        shootingState = ShootingState.IDLE;
        ie.setPower(0);
    }
    
    public void shoot(){
        ShotsRemaining = 4;
    }

    public ShootingState getState() {
        return shootingState;
    }

    public void update() {
        boolean atSpeed =
                Math.abs(Lsh.getVelocity() - CLOSE_VELOCITY) <= VELOCITY_TOLERANCE &&
                        Math.abs(Rsh.getVelocity() - CLOSE_VELOCITY) <= VELOCITY_TOLERANCE;

        switch (shootingState) {
            case IDLE:
                if (ShotsRemaining > 0) {
                    Lfeeder.setPower(off);
                    Rfeeder.setPower(off);
                    ie.setPower(ieP);
                    Lsh.setVelocity(CLOSE_VELOCITY);
                    Rsh.setVelocity(CLOSE_VELOCITY);

                    stateTimer.reset();
                    shootingState = ShootingState.SPIN_UP;
                } else {
                    Lfeeder.setPower(off);
                    Rfeeder.setPower(off);
                    ie.setPower(0);
                }
                break;

            case SPIN_UP:
                if (atSpeed || stateTimer.milliseconds() >= SPIN_TIME) {
                    Lfeeder.setPower(LfeederP);
                    stateTimer.reset();
                    shootingState = ShootingState.LAUNCH1;
                }
                break;

            case LAUNCH1:
                if (stateTimer.milliseconds() >= LfeederT) {
                    Lfeeder.setPower(0.0);
                    stateTimer.reset();
                    Rfeeder.setPower(RfeederP);
                    shootingState = ShootingState.LAUNCH2;
                    ShotsRemaining--;
                }
                break;

            case LAUNCH2:
                if (stateTimer.milliseconds() >= RfeederT) {
                    Rfeeder.setPower(0.0);
                    stateTimer.reset();
                    Lfeeder.setPower(LfeederP);
                    shootingState = ShootingState.LAUNCH3;
                    ShotsRemaining--;
                }
                break;

            case LAUNCH3:
                if (stateTimer.milliseconds() >= LfeederT) {
                    Lfeeder.setPower(0.0);
                    stateTimer.reset();
                    Rfeeder.setPower(RfeederP);
                    shootingState = ShootingState.LAUNCH4;
                    ShotsRemaining--;
                }
                break;

            case LAUNCH4:
                if (stateTimer.milliseconds() >= RfeederT) {
                    Rfeeder.setPower(0.0);
                    stateTimer.reset();
                    shootingState = ShootingState.IDLE;
                    ShotsRemaining--;
                }
                break;

        }
    }
}
