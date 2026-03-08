package org.firstinspires.ftc.teamcode;
import static org.firstinspires.ftc.teamcode.shooter.ShootingState.IDLE;
import static org.firstinspires.ftc.teamcode.shooter.ShootingStateFar.IDLEF;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

public class shooter {
    public static CRServo Lfeeder, Rfeeder;
    public static DcMotorEx Lsh, Rsh;
    private static DcMotorEx Lf;
    private static DcMotorEx Rf;
    private static DcMotorEx Lb;
    private static DcMotorEx Rb;
    public static DcMotorEx ie;
    public static Servo ll;
    public static Servo lr;
    public static boolean far = false;
    public static boolean close = false;
    public ElapsedTime stateTimer = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);

    public enum ShootingState {
        IDLE2,
        IDLE,
        SPIN_UP,
        LAUNCH1,
        LAUNCH2,
        LAUNCH3,
        LAUNCH4,

    }

    public enum ShootingStateFar {
        IDLE2F,
        IDLEF,
        SPIN_UPF,
        LAUNCH1F,
        LAUNCH2F,
        LAUNCH3F,
        LAUNCH4F,

    }

    public static ShootingState shootingState = IDLE;
    public static ShootingStateFar shootingstatefar = IDLEF;

    // FEEDER / INTAKE
    public static double LfeederP = 1.0;
    public static double RfeederP = 1.0;
    public static double LfeederT = 500;
    public static double RfeederT = 500;
    public static double ieP = .80;
    public static double off = -0.2;
    public static double offW = 0.0;
    public static double F = 14;
    public static double P = 400;
    public static double red = 0.3;
    public static double green = .5;

    // SHOOTER
    public static double CLOSE_VELOCITY = 1110;
    public static double FAR_VELOCITY = 1425;
    public static double VELOCITY_TOLERANCE = 20;
    public static double SPIN_TIME = 500;
    public static double SPIN_TIMEFAR = 2000;


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
        Lsh.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        Rsh.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        Rfeeder.setDirection(DcMotorSimple.Direction.REVERSE);

        PIDFCoefficients pidfCoefficients = new PIDFCoefficients(P, 0, 0, F);
        Lsh.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidfCoefficients);
        Rsh.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidfCoefficients);

        shootingState = IDLE;
        ie.setPower(0);

    }

    public void light() {
        if (ShotsRemaining == 0) {
            ll.setPosition(red);
            lr.setPosition(red);
        }
    }

    public void shoot() {
        ShotsRemaining = 4;
        close = true;
    }

    public void shootFar() {
        ShotsRemaining = 4;
        far = true;
    }

    public ShootingState getState() {
        return shootingState;
    }

    public void update() {
        light();
        boolean atSpeed =
                Math.abs(Lsh.getVelocity() - CLOSE_VELOCITY) <= VELOCITY_TOLERANCE &&
                        Math.abs(Rsh.getVelocity() - CLOSE_VELOCITY) <= VELOCITY_TOLERANCE;
        boolean atSpeedFar =
                Math.abs(Lsh.getVelocity() - FAR_VELOCITY) <= VELOCITY_TOLERANCE &&
                        Math.abs(Rsh.getVelocity() - FAR_VELOCITY) <= VELOCITY_TOLERANCE;

        switch (shootingState) {
            case IDLE:
                if (shootingState == ShootingState.IDLE && shootingstatefar == IDLEF){
                    Lfeeder.setPower(off);
                    Rfeeder.setPower(off);
                }
                if (ShotsRemaining > 0 && close == true) {
                    Lfeeder.setPower(off);
                    Rfeeder.setPower(off);
                    ie.setPower(ieP);
                    Lsh.setVelocity(CLOSE_VELOCITY);
                    Rsh.setVelocity(CLOSE_VELOCITY);
                    ll.setPosition(green);
                    lr.setPosition(green);

                    stateTimer.reset();
                    shootingState = ShootingState.SPIN_UP;
                }
                break;

            case SPIN_UP:
                if (atSpeed || stateTimer.milliseconds() >= SPIN_TIME) {
                    ie.setPower(ieP);
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
                    shootingState = IDLE;
                    ShotsRemaining--;
                }
                break;

        }

        switch (shootingstatefar) {
            case IDLEF:
                if (ShotsRemaining > 0 && far == true ) {
                    Lfeeder.setPower(off);
                    Rfeeder.setPower(off);
                    ie.setPower(ieP);
                    Lsh.setVelocity(FAR_VELOCITY);
                    Rsh.setVelocity(FAR_VELOCITY);
                    ll.setPosition(green);
                    lr.setPosition(green);

                    stateTimer.reset();
                    shootingstatefar = ShootingStateFar.SPIN_UPF;
                }
                break;

            case SPIN_UPF:
                if (atSpeed || stateTimer.milliseconds() >= SPIN_TIMEFAR) {
                    ie.setPower(ieP);
                    Lfeeder.setPower(LfeederP);
                    stateTimer.reset();
                    shootingstatefar = ShootingStateFar.LAUNCH1F;
                }
                break;

            case LAUNCH1F:
                if (stateTimer.milliseconds() >= LfeederT) {
                    Lfeeder.setPower(0.0);
                    stateTimer.reset();
                    Rfeeder.setPower(RfeederP);
                    shootingstatefar = ShootingStateFar.LAUNCH2F;
                    ShotsRemaining--;
                }
                break;

            case LAUNCH2F:
                if (stateTimer.milliseconds() >= RfeederT) {
                    Rfeeder.setPower(0.0);
                    stateTimer.reset();
                    Lfeeder.setPower(LfeederP);
                    shootingstatefar = ShootingStateFar.LAUNCH3F;
                    ShotsRemaining--;
                }
                break;

            case LAUNCH3F:
                if (stateTimer.milliseconds() >= LfeederT) {
                    Lfeeder.setPower(0.0);
                    stateTimer.reset();
                    Rfeeder.setPower(RfeederP);
                    shootingstatefar = ShootingStateFar.LAUNCH4F;
                    ShotsRemaining--;
                }
                break;

            case LAUNCH4F:
                if (stateTimer.milliseconds() >= RfeederT) {
                    Rfeeder.setPower(0.0);
                    stateTimer.reset();
                    shootingstatefar = IDLEF;
                    far = false;
                    ShotsRemaining--;
                }
                break;

        }
    }
}
