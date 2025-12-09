package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.dfrobot.HuskyLens;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.internal.system.Deadline;

import java.util.concurrent.TimeUnit;

@Config
@TeleOp(name = "newest sorting 2")
public class NewestSorting2 extends OpMode {
    final int READ_PERIOD = 1;
    DcMotor intakeMotor;
    Servo pitchServo;
    DcMotor rotationMotor;
    DcMotor shootingMotor;
    DcMotor sorterMotor;
    Servo leverServo;
    HuskyLens huskyLens;
    HuskyLens huskyLens2;
    IMU turretImu;
    Deadline rateLimit;
    ElapsedTime timer = new ElapsedTime();

    boolean prevPressed = false;
    public static int encoderAmount = 18;

    public static int timeAmount = 70;

    @Override
    public void init() {
        intakeMotor = hardwareMap.get(DcMotor.class, "intakeMotor");
        pitchServo = hardwareMap.get(Servo.class,"pitchServo");
        rotationMotor = hardwareMap.get(DcMotor.class, "rotationMotor");
        shootingMotor = hardwareMap.get(DcMotor.class, "shootingMotor");
        huskyLens = hardwareMap.get(HuskyLens.class, "huskylens");
        huskyLens2 = hardwareMap.get(HuskyLens.class, "huskylens2");
        turretImu = hardwareMap.get(IMU.class, "turretImu");

        // Set up parameters for turret orientation (adjust based on mounting)
        RevHubOrientationOnRobot orientationOnRobot = new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.RIGHT,
                RevHubOrientationOnRobot.UsbFacingDirection.UP
        );

        // Initialize
        turretImu.initialize(new IMU.Parameters(orientationOnRobot));

        sorterMotor = hardwareMap.get(DcMotor.class, "sorterMotor");
        leverServo = hardwareMap.get(Servo.class,"leverServo");
        intakeMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        intakeMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rotationMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rotationMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        sorterMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        // Rate limiter for telemetry
        rateLimit = new Deadline(READ_PERIOD, TimeUnit.SECONDS);
        rateLimit.expire();
        leverServo.setPosition(0);
        // Choose the algorithm
        huskyLens.selectAlgorithm(HuskyLens.Algorithm.TAG_RECOGNITION);
        // huskyLens.selectAlgorithm(HuskyLens.Algorithm.COLOR_RECOGNITION);
        timer.reset();
    }

    @Override
    public void start() {
        timer.reset();
    }

    @Override
    public void loop() {
        boolean aPressed = gamepad1.a;


        if (aPressed && !prevPressed) {
            int pos = sorterMotor.getCurrentPosition();
            pos += encoderAmount;
            sorterMotor.setTargetPosition(pos);
            sorterMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            sorterMotor.setPower(0.75);
            if (sorterMotor.getCurrentPosition() == pos) {
                sorterMotor.setPower(0);
            }
        }
        prevPressed = aPressed;

        telemetry.addData("encoder ticks", sorterMotor.getCurrentPosition());

    }
}