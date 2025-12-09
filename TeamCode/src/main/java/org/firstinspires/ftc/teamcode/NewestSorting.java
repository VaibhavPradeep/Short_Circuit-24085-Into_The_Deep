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
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import java.util.concurrent.TimeUnit;

@Config
@TeleOp(name = "newest sorting ")
public class NewestSorting extends OpMode {
    final int READ_PERIOD = 1;
    DcMotor intakeMotor;
    Servo pitchServo;
    DcMotor rotationMotor;
    DcMotor shootingMotor;
    DcMotor sorterMotor;
    Servo leverServo;
    //ColorSensor colorSensor;
    HuskyLens huskyLens;
    HuskyLens huskyLens2;
    IMU turretImu;
    Deadline rateLimit;
    public static int encoderAmount = 18;
    boolean sorting = false;
    ElapsedTime timer = new ElapsedTime();

    boolean prevPressed = false;

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
                RevHubOrientationOnRobot.LogoFacingDirection.UP,
                RevHubOrientationOnRobot.UsbFacingDirection.RIGHT
        );

        // Initialize
        turretImu.initialize(new IMU.Parameters(orientationOnRobot));


        sorterMotor = hardwareMap.get(DcMotor.class, "sorterMotor");
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
        /*
        if (!rateLimit.hasExpired()) {
            return;
        }
        rateLimit.reset();
        // Get blocks (recognized objects)
        HuskyLens.Block[] blocks = huskyLens2.blocks();
        telemetry.addData("Block count", blocks.length);
        int currentPos = intakeMotor.getCurrentPosition();
        int newPos = currentPos + encoderAmount;

         */


        // When X is pressed, start a sort cycle
        boolean aPressed = gamepad1.a;

        if (aPressed && !prevPressed) {
            int currentPos = intakeMotor.getCurrentPosition();
            int newPos = currentPos + encoderAmount;
            intakeMotor.setTargetPosition(newPos);
            intakeMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        }

        prevPressed = aPressed;




        /*
        if (detectGreen() || detectPurple()) {
            telemetry.addLine(" Ready to transfer!");
        }

        telemetry.addData("encoder ticks", intakeMotor.getCurrentPosition());

         */
    }

    /*
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
     */
}