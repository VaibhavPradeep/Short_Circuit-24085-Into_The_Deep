package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.dfrobot.HuskyLens;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.internal.system.Deadline;

import java.util.concurrent.TimeUnit;

@Config
@TeleOp(name = "no sorting husky test")
public class NoSortingHuskyTest extends OpMode {

    /* Option 1:
    divide total time needed into 6, so we get 60 degree rotations,
    color sensor to detect the ball color
    manual button to run it to normal position
     */

    /*
    Option 2:
    Husky lens: object symbol alignment
    color sensor detection
     */

    /*
    Option 3:
    thru bore encoder: into sixths,
    color sensor detection
     */


    public static double transferElapseTime = 0;
    ElapsedTime timer = new ElapsedTime();
    public enum MotifCode{
        NONE,
        TAG_ID_1_GPP,
        TAG_ID_2_PGP,
        TAG_ID_3_PPG

    };

    public enum Motif1GPP{
        G,
        P1,
        P2
    }

    public enum Motif2PGP{
        P1,
        G,
        P2
    }

    public enum Motif3PPG{
        P1,
        P2,
        G
    }

    boolean sendUp = false;
    boolean trackingRed = false;
    MotifCode motifCode = MotifCode.TAG_ID_2_PGP;
    Motif1GPP motif1 = Motif1GPP.G;
    Motif2PGP motif2 = Motif2PGP.P1;
    Motif3PPG motif3 = Motif3PPG.P1;
    
    final int READ_PERIOD = 1;

    // TODO: find lever pos
    DcMotor intakeMotor;
    Servo pitchServo;
    DcMotor rotationMotor;
    DcMotor shootingMotor;

    DcMotor transferMotor;
    CRServo sorterServo;
    Servo leverServo;

    ColorSensor colorSensor;
    HuskyLens huskyLens;
    HuskyLens huskyLens2;
    Deadline rateLimit;

    public static double leverPos = 0;
    public static double transferMotorPower = 0;
    public static double sorterServoPower = 0;
    public static double intakeMotorPower = 0;
    
    @Override
    public void init() {
        intakeMotor = hardwareMap.get(DcMotor.class, "intakeMotor");
        pitchServo = hardwareMap.get(Servo.class,"pitchServo");
        rotationMotor = hardwareMap.get(DcMotor.class, "rotationMotor");
        shootingMotor = hardwareMap.get(DcMotor.class, "shootingMotor");
        colorSensor = hardwareMap.get(ColorSensor.class,"colorSensor");
        huskyLens = hardwareMap.get(HuskyLens.class, "huskylens");
        huskyLens2 = hardwareMap.get(HuskyLens.class, "huskylens2");

        transferMotor = hardwareMap.get(DcMotor.class, "transferMotor");
        sorterServo = hardwareMap.get(CRServo.class, "sorterServo");
        leverServo = hardwareMap.get(Servo.class,"leverServo");

        intakeMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        leverServo.setDirection(Servo.Direction.REVERSE);
        rotationMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rotationMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Rate limiter for telemetry
        rateLimit = new Deadline(READ_PERIOD, TimeUnit.SECONDS);
        rateLimit.expire();

        leverServo.setPosition(0);

        // Choose the algorithm
        huskyLens2.selectAlgorithm(HuskyLens.Algorithm.OBJECT_RECOGNITION);
        // huskyLens.selectAlgorithm(HuskyLens.Algorithm.COLOR_RECOGNITION);
    }

    @Override
    public void loop() {
        transferMotor.setPower(transferMotorPower);
        shootingMotor.setPower(1);
        intakeMotor.setPower(intakeMotorPower);

        HuskyLens.Block[] blocks = huskyLens2.blocks();
        telemetry.addData("Block count", blocks.length);

        if (intakeMotor.getPower() == 1) {
            sorterServo.setPower(-0.5);
            huskyLens2.selectAlgorithm(HuskyLens.Algorithm.OBJECT_RECOGNITION);
            trackingRed = true;
        }

        for (int i = 0; i < blocks.length; i++) {
            telemetry.addData("Block", blocks[i].toString());
            if (blocks[i].id == 1 && trackingRed) {
                sorterServo.setPower(0);
            } else if (blocks[i].id == 1 && gamepad1.b) {
                sendUp = true;
            }
        }

        if (sendUp) {
            timer.reset();
            if (timer.milliseconds() == transferElapseTime) {
                leverOff();
                leverOn();
                timer.reset();
                if (timer.milliseconds() == 500) {
                    leverOff();
                }
            }
        }


        /*
        if (detectGreen()) {
            timer.reset();
            if (timer.milliseconds() == transferElapseTime) {

                leverOff();
                leverOn();
                leverOff();
            }
        }

        if (motifCode == MotifCode.TAG_ID_1_GPP) {
            switch(motif1) {
                case G:
                    if (detectGreen()) {
                        timer.reset();
                        if (timer.milliseconds() == transferElapseTime) {

                            leverOff();
                            leverOn();
                            leverOff();

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

                            motif1 = Motif1GPP.G;
                        }
                    }
                    break;
            }
        } else if (motifCode == MotifCode.TAG_ID_2_PGP) {
            switch(motif2) {
                case P1:
                    if (detectPurple()) {
                        timer.reset();
                        if (timer.milliseconds() == transferElapseTime) {

                            leverOff();
                            leverOn();
                            leverOff();

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

                            motif2 = Motif2PGP.P1;
                        }
                    }
                    break;
            }
        } else if (motifCode == MotifCode.TAG_ID_3_PPG) {
            switch(motif3) {
                case P1:
                    if (detectPurple()) {
                        timer.reset();
                        if (timer.milliseconds() == transferElapseTime) {

                            leverOff();
                            leverOn();
                            leverOff();

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

                            motif3 = Motif3PPG.P1;
                        }
                    }
                    break;
                default:
                    motif3 = Motif3PPG.P1;
            }

        }

         */
    }


    public boolean detectGreen() {
        if (colorSensor.green() >= 150) {
            return true;
        } else {
            return false;
        }
    }

    public void leverOn(){
        leverServo.setPosition(0);
    }

    public void leverOff() {
        leverServo.setPosition(leverPos);
    }

    public boolean detectPurple() {
        if (colorSensor.red() >= 140 && colorSensor.blue() >= 125) {
            return true;
        } else {
            return false;
        }
    }
}
