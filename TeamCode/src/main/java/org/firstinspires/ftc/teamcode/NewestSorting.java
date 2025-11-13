package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.bosch.BNO055IMU;
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
@TeleOp(name = "newest sorting ")
public class NewestSorting extends OpMode {
    final int READ_PERIOD = 1;
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
    BNO055IMU turretImu;
    Deadline rateLimit;
    public static int encoderAmount = 0;

    boolean detected = false;

    ElapsedTime timer = new ElapsedTime();
    @Override
    public void init() {
        intakeMotor = hardwareMap.get(DcMotor.class, "intakeMotor");
        pitchServo = hardwareMap.get(Servo.class,"pitchServo");
        rotationMotor = hardwareMap.get(DcMotor.class, "rotationMotor");
        shootingMotor = hardwareMap.get(DcMotor.class, "shootingMotor");
        colorSensor = hardwareMap.get(ColorSensor.class,"colorSensor");
        huskyLens = hardwareMap.get(HuskyLens.class, "huskylens");
        huskyLens2 = hardwareMap.get(HuskyLens.class, "huskylens2");


        rotationMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rotationMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        transferMotor = hardwareMap.get(DcMotor.class, "transferMotor");
        sorterServo = hardwareMap.get(CRServo.class, "sorterServo");
        leverServo = hardwareMap.get(Servo.class,"leverServo");

        rotationMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rotationMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Rate limiter for telemetry
        rateLimit = new Deadline(READ_PERIOD, TimeUnit.SECONDS);
        rateLimit.expire();

        leverServo.setPosition(0);

        // Choose the algorithm
        huskyLens.selectAlgorithm(HuskyLens.Algorithm.TAG_RECOGNITION);
        huskyLens2.selectAlgorithm(HuskyLens.Algorithm.OBJECT_RECOGNITION);
        // huskyLens.selectAlgorithm(HuskyLens.Algorithm.COLOR_RECOGNITION);
        timer.reset();
    }

    @Override
    public void loop() {

        if (!rateLimit.hasExpired()) {
            return;
        }
        rateLimit.reset();

        // Get blocks (recognized objects)
        HuskyLens.Block[] blocks = huskyLens.blocks();
        telemetry.addData("Block count", blocks.length);

        int currentPos = rotationMotor.getCurrentPosition();
        int newPos = currentPos + encoderAmount;

        if (gamepad1.a) {
            while (rotationMotor.getCurrentPosition() <= newPos) {
                sorterServo.setPower(0.5);
            }
        }

        if (detectGreen() || detectPurple()) {
            telemetry.addLine(" Ready to transfer!");
        }

        if (gamepad1.b) {
            for (int i = 0; i < blocks.length; i++) {
                telemetry.addData("Block", blocks[i].toString());
                if (blocks[i].id == 1) {
                    detected = true;
                    while (blocks[i].x != 0) {
                        sorterServo.setPower(0.5);
                    }
                }
                // Example of accessing block fields:
                // blocks[i].x, blocks[i].y, blocks[i].width, blocks[i].height, blocks[i].id
            }
        }


        telemetry.addData("encoder ticks", rotationMotor.getCurrentPosition());



    }

    public boolean detectGreen() {
        if (colorSensor.green() >= 150) {
            return true;
        } else {
            return false;
        }
    }

    public boolean detectPurple() {
        if (colorSensor.red() >= 100 && colorSensor.blue() >= 100) {
            return true;
        } else {
            return false;
        }
    }
}
