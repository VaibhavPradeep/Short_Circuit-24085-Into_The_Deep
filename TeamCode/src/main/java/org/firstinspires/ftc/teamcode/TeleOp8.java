package org.firstinspires.ftc.teamcode;


import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.dfrobot.HuskyLens;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.internal.system.Deadline;

import java.util.concurrent.TimeUnit;

// @TeleOp(name = "Teleop 8")
public class TeleOp8 extends OpMode {

    final int READ_PERIOD = 1;

    // TODO: find lever pos
    double leverPos = 0.5;
    public static int encoderAmount = 0;
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

    IMU imu;
    BNO055IMU turretImu;
    Deadline rateLimit;
    DcMotor frontLeft;
    DcMotor frontRight;
    DcMotor backLeft;
    DcMotor backRight;

    ElapsedTime timer = new ElapsedTime();

    @Override
    public void init() {


        intakeMotor = hardwareMap.get(DcMotor.class, "intakeMotor");
        pitchServo = hardwareMap.get(Servo.class,"pitchServo");
        rotationMotor = hardwareMap.get(DcMotor.class, "rotationMotor");
        shootingMotor = hardwareMap.get(DcMotor.class, "shootingMotor");
        colorSensor = hardwareMap.get(ColorSensor.class,"colorSensor");
        imu = hardwareMap.get(IMU.class, "imu");
        turretImu = hardwareMap.get(BNO055IMU.class, "turretImu");

        huskyLens = hardwareMap.get(HuskyLens.class, "huskylens");
        huskyLens2 = hardwareMap.get(HuskyLens.class, "huskylens2");

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

        frontLeft = hardwareMap.get(DcMotor.class, "frontLeft");
        frontRight = hardwareMap.get(DcMotor.class, "frontRight");
        backLeft = hardwareMap.get(DcMotor.class, "backLeft");
        backRight = hardwareMap.get(DcMotor.class, "backRight");

        frontRight.setDirection(DcMotorSimple.Direction.REVERSE);
        backRight.setDirection(DcMotorSimple.Direction.REVERSE);
        frontLeft.setDirection(DcMotorSimple.Direction.FORWARD);
        backLeft.setDirection(DcMotorSimple.Direction.FORWARD);

        intakeMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        pitchServo.setDirection(Servo.Direction.REVERSE);

        frontLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        frontRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        frontLeft.setPower(0);
        frontRight.setPower(0);
        backLeft.setPower(0);
        backRight.setPower(0);

        leverServo.setPosition(leverPos);
    }

    @Override
    public void loop() {
        driveMecanum(gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x);

        if (gamepad1.a) {
            intakeMotor.setPower(1);
        }
        if (gamepad1.b) {
            intakeMotor.setPower(0);
        }

        if (gamepad2.x) {
            shootingMotor.setPower(1);
        }
        if (gamepad2.y) {
            shootingMotor.setPower(0);
        }

        if (gamepad2.left_bumper) {
            transferMotor.setPower(1);
        }
        if (gamepad2.right_bumper) {
            transferMotor.setPower(-1);
        }

        if (!rateLimit.hasExpired()) {
            return;
        }
        rateLimit.reset();

        // Get blocks (recognized objects)
        HuskyLens.Block[] blocks = huskyLens2.blocks();
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
            leverServo.setPosition(0);
            timer.reset();
            if (timer.milliseconds() > 500) {
                leverServo.setPosition(leverPos);
            }
        }

        if (gamepad1.b) {
            for (int i = 0; i < blocks.length; i++) {
                telemetry.addData("Block", blocks[i].toString());
                if (blocks[i].id == 1) {
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

    public void driveMecanum(double left_y, double left_x, double right_x){
        double maxPower = Math.max(Math.abs(left_y) + Math.abs(left_x) + Math.abs(right_x), 1);
        frontLeft.setPower((left_y - left_x - right_x) / maxPower);
        frontRight.setPower((left_y + left_x + right_x) / maxPower);
        backLeft.setPower((left_y + left_x - right_x) / maxPower);
        backRight.setPower((left_y - left_x + right_x) / maxPower);
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
