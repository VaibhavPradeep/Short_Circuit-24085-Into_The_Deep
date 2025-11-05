package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.dfrobot.HuskyLens;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.internal.system.Deadline;

import java.util.concurrent.TimeUnit;

public class NewILC {
    final int READ_PERIOD = 1;

    // TODO: find lever pos
    double leverPos = 0.5;
    DcMotor intakeMotor;
    DcMotor pitchMotor;
    DcMotor rotationMotor;
    DcMotor shootingMotor;

    DcMotor transferMotor;
    CRServo sorterServo;
    Servo leverServo;

    ColorSensor colorSensor;
    HuskyLens huskyLens;
    Deadline rateLimit;



    public void initILC(HardwareMap hwMap, Telemetry telemetry) {
        intakeMotor = hwMap.get(DcMotor.class, "intakeMotor");
        pitchMotor = hwMap.get(DcMotor.class, "pitchMotor");
        rotationMotor = hwMap.get(DcMotor.class, "rotationMotor");
        shootingMotor = hwMap.get(DcMotor.class, "shootingMotor");
        colorSensor = hwMap.get(ColorSensor.class,"colorSensor");
        huskyLens = hwMap.get(HuskyLens.class, "huskylens");

        transferMotor = hwMap.get(DcMotor.class, "transferMotor");
        sorterServo = hwMap.get(CRServo.class, "sorterServo");
        leverServo = hwMap.get(Servo.class,"leverServo");

        rotationMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        pitchMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rotationMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Rate limiter for telemetry
        rateLimit = new Deadline(READ_PERIOD, TimeUnit.SECONDS);
        rateLimit.expire();

        leverServo.setPosition(0);

        // Choose the algorithm
        huskyLens.selectAlgorithm(HuskyLens.Algorithm.TAG_RECOGNITION);
        // huskyLens.selectAlgorithm(HuskyLens.Algorithm.COLOR_RECOGNITION);

    }

    public void intakeOn(){
        intakeMotor.setPower(1);
    }

    public void intakeOff(){
        intakeMotor.setPower(0);
    }

    public void verticalTurretManualMove(String direction) {
        int Pos = pitchMotor.getCurrentPosition();

        if(direction.equals("up")) {
            Pos += 100;
        }
        else if(direction.equals("down")) {Pos -= 100;
        }

        pitchMotor.setTargetPosition(Pos);
        pitchMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        pitchMotor.setPower(1);

        if (pitchMotor.getCurrentPosition() == Pos) {
            pitchMotor.setPower(0);
        }
    }

    public void lateralTurretManualMove(String direction) {
        int Pos = rotationMotor.getCurrentPosition();

        if(direction.equals("left")) {
            Pos += 100;
        }
        else if(direction.equals("right")) {Pos -= 100;
        }

        rotationMotor.setTargetPosition(Pos);
        rotationMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rotationMotor.setPower(1);

        if (rotationMotor.getCurrentPosition() == Pos) {
            rotationMotor.setPower(0);
        }
    }

    public void transferOn() {
        transferMotor.setPower(1);
    }

    public void transferOff() {
        transferMotor.setPower(0);
    }

    public void sorterOn() {
        sorterServo.setPower(1);
    }

    public void sorterOff() {
        sorterServo.setPower(0);
    }

    public void shootingOn(){
        shootingMotor.setPower(1);
    }

    public void shootingOff(){
        shootingMotor.setPower(0);
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
        leverServo.setPosition(leverPos);
    }

    public boolean detectPurple() {
        if (colorSensor.red() >= 140 && colorSensor.blue() >= 125) {
            return true;
        } else {
            return false;
        }
    }

    public void rateLimit() {
        if (!rateLimit.hasExpired()) {
            return;
        }
        rateLimit.reset();
    }
}
