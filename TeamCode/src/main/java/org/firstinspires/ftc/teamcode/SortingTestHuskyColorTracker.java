package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.dfrobot.HuskyLens;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.internal.system.Deadline;

import java.util.concurrent.TimeUnit;

@Config
@TeleOp(name = "sorting test husky color tracker dont run")
public class SortingTestHuskyColorTracker extends OpMode {
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
        huskyLens = hardwareMap.get(HuskyLens.class, "huskyLens");

        transferMotor = hardwareMap.get(DcMotor.class, "transferMotor");
        sorterServo = hardwareMap.get(CRServo.class, "sorterServo");
        leverServo = hardwareMap.get(Servo.class,"leverServo");

        leverServo.setDirection(Servo.Direction.REVERSE);
        rotationMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rotationMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Rate limiter for telemetry
        rateLimit = new Deadline(READ_PERIOD, TimeUnit.SECONDS);
        rateLimit.expire();

        leverServo.setPosition(0);

        // Choose the algorithm
        huskyLens.selectAlgorithm(HuskyLens.Algorithm.TAG_RECOGNITION);
        // huskyLens.selectAlgorithm(HuskyLens.Algorithm.COLOR_RECOGNITION);
    }

    @Override
    public void loop() {
        transferMotor.setPower(transferMotorPower);
        sorterServo.setPower(sorterServoPower);
        intakeMotor.setPower(intakeMotorPower);


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
    }


    public boolean detectGreen() {
        if (colorSensor.green() >= 150) {
            return true;
        } else {
            return false;
        }
    }

    public void leverOff(){
        leverServo.setPosition(0);
    }

    public void leverOn() {
        leverServo.setPosition(0.22);
    }

    public boolean detectPurple() {
        if (colorSensor.red() >= 140 && colorSensor.blue() >= 125) {
            return true;
        } else {
            return false;
        }
    }
}
