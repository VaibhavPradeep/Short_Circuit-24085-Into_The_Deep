package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.hardware.dfrobot.HuskyLens;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.internal.system.Deadline;
/*
public class AutosortTest extends OpMode{


    // 6500 rpm is max, at far range 0.35 pitch pos
    // 0.3, 4600

    public static double targetPower = 0;

    public static double LONG_SHOT = 0.87;

    public static double SHORT_SHOT = 0.795;

    public static double LONG_SERVO_POS = 0.3;
    public static double SHORT_SERVO_POS = 0.4;

    // end


    private DcMotor sorterMotor;
    private PIDController pid;

    final int READ_PERIOD = 1;

    private static final double TICKS_PER_REV = 537.6;  // 312 RPM motor

    // Positions around sorterMotor circle (in degrees)
    // CHANGE THESE BASED ON YOUR REAL sorterMotor GEOMETRY
    private double[] positionDegrees = { 0, 60, 120, 180, 240, 300 };

    private int currentIndex = 0;
    private int targetTicks = 0;

    // PID gains — tune as needed
    private double kP = 0.0029;
    // p=0.0065,kS = 0.025, d=0.0003
    private double kI = 0.0;
    private double kD = 0.00017;

    // <-- ADDED: static feedforward (kS)
    public static double kS = 0.02;

    // For button debounce
    private long lastAdvanceTime = 0;
    private static final long ADVANCE_DEBOUNCE_MS = 200;

    DcMotor intakeMotor;
    Servo pitchServo;
    DcMotor rotationMotor;
    DcMotorEx shootingMotor;
    Servo leverServo;
    ColorSensor colorSensor;
    HuskyLens huskyLens;
    HuskyLens huskyLens2;
    Deadline rateLimit;
    DcMotor frontLeft;
    DcMotor frontRight;
    DcMotor backLeft;
    DcMotor backRight;
    IMU turretImu;

    boolean prevX = false;
    boolean sorting = false;

    public static int timeAmount = 110;
    ElapsedTime timer = new ElapsedTime();
    public static double leverPos = 0;
    public static double pitchPos = 0.42;

    public static int encoderAmount = 89;

    boolean prevPressed = false;
    
    double transferElapseTime = 0;
    public enum MotifCode{
        NONE,
        TAG_ID_1_GPP,
        TAG_ID_2_PGP,
        TAG_ID_3_PPG

    };

    enum Motif1GPP{
        G,
        P1,
        P2
    }

    enum Motif2PGP{
        P1,
        G,
        P2
    }

    enum Motif3PPG{
        P1,
        P2,
        G
    }

    MotifCode motifCode = MotifCode.NONE;
    Motif1GPP motif1 = Motif1GPP.G;
    Motif2PGP motif2 = Motif2PGP.P1;
    Motif3PPG motif3 = Motif3PPG.P1;


    @Override
    public void init() {
        

    }

    @Override
    public void loop() {


        // Get blocks and husky lens data
        HuskyLens.Block[] blocks = huskyLens.blocks();
        telemetry.addData("Block count", blocks.length);

        while (motifCode == MotifCode.NONE) {
            for (int i = 0; i < blocks.length; i++) {
                telemetry.addData("Block", blocks[i].toString());
                if (blocks[i].id == 1) {
                    MotifCode motifCode = MotifCode.TAG_ID_1_GPP;
                    gamepad1.rumble(500);
                    gamepad2.rumble(500);
                } else if (blocks[i].id == 2) {
                    MotifCode motifCode = MotifCode.TAG_ID_2_PGP;
                    gamepad1.rumble(500);
                    gamepad2.rumble(500);
                } else if (blocks[i].id == 3) {
                    MotifCode motifCode = MotifCode.TAG_ID_3_PPG;
                    gamepad1.rumble(500);
                    gamepad2.rumble(500);
                }
            }
        }

        if (motifCode == MotifCode.TAG_ID_1_GPP) {
            switch (motif1) {
                case G:
                    if (detectGreen()) {
                        timer.reset();
                        if (timer.milliseconds() == transferElapseTime) {

                            leverOff();
                            leverOn();
                            leverOff();
                            transferOn();
                            motif1 = Motif1GPP.P1;
                        }
                    }
                    break;
                case P1:
                    if (detectPurple()) {
                        timer.reset();
                        if (timer.milliseconds() == transferElapseTime) {

                            leverOff();
                            leverOn();
                            leverOff();
                            transferOn();
                            motif1 = Motif1GPP.P2;
                        }
                    }
                    break;
                case P2:
                    if (detectPurple()) {
                        timer.reset();
                        if (timer.milliseconds() == transferElapseTime) {

                            leverOff();
                            leverOn();
                            leverOff();
                            transferOn();
                            motif1 = Motif1GPP.G;
                        }
                    }
                    break;
            }
        } else if (motifCode == MotifCode.TAG_ID_2_PGP) {
            switch (motif2) {
                case P1:
                    if (detectPurple()) {
                        timer.reset();
                        if (timer.milliseconds() == transferElapseTime) {

                            leverOff();
                            leverOn();
                            leverOff();
                            transferOn();
                            motif2 = Motif2PGP.G;
                        }
                    }
                    break;
                case G:
                    if (detectGreen()) {
                        timer.reset();
                        if (timer.milliseconds() == transferElapseTime) {

                            leverOff();
                            leverOn();
                            leverOff();
                            transferOn();
                            motif2 = Motif2PGP.P2;
                        }
                    }
                    break;
                case P2:
                    if (detectPurple()) {
                        timer.reset();
                        if (timer.milliseconds() == transferElapseTime) {

                            leverOff();
                            leverOn();
                            leverOff();
                            transferOn();
                            motif2 = Motif2PGP.P1;
                        }
                    }
                    break;
            }
        } else if (motifCode == MotifCode.TAG_ID_3_PPG) {
            switch (motif3) {
                case P1:
                    if (detectPurple()) {
                        timer.reset();
                        if (timer.milliseconds() == transferElapseTime) {

                            leverOff();
                            leverOn();
                            leverOff();
                            transferOn();
                            motif3 = Motif3PPG.P2;
                        }
                    }
                    break;
                case P2:
                    if (detectPurple()) {
                        timer.reset();
                        if (timer.milliseconds() == transferElapseTime) {

                            leverOff();
                            leverOn();
                            leverOff();
                            transferOn();
                            motif3 = Motif3PPG.G;
                        }
                    }
                    break;
                case G:
                    if (detectGreen()) {
                        timer.reset();
                        if (timer.milliseconds() == transferElapseTime) {

                            leverOff();
                            leverOn();
                            leverOff();
                            transferOn();
                            motif3 = Motif3PPG.P1;
                        }
                    }
                    break;
                default:
                    motif3 = Motif3PPG.P1;
            }

        }


    }

    private void waitMs(long ms) {
        ElapsedTime timer = new ElapsedTime();
        timer.reset();
        while (timer.milliseconds() < ms) {
        }
    }
}


 */