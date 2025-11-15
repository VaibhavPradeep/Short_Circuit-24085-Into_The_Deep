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
@TeleOp(name = "newest sorting test")
public class NewestSortingTest extends OpMode {
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
    IMU turretImu;
    Deadline rateLimit;
    ElapsedTime timer = new ElapsedTime();

    boolean prevX = false;
    boolean sorting = false;

    public static int timeAmount = 70;

    @Override
    public void init() {
        intakeMotor = hardwareMap.get(DcMotor.class, "intakeMotor");
        pitchServo = hardwareMap.get(Servo.class,"pitchServo");
        rotationMotor = hardwareMap.get(DcMotor.class, "rotationMotor");
        shootingMotor = hardwareMap.get(DcMotor.class, "shootingMotor");
        colorSensor = hardwareMap.get(ColorSensor.class,"colorSensor");
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

        transferMotor = hardwareMap.get(DcMotor.class, "transferMotor");
        sorterServo = hardwareMap.get(CRServo.class, "sorterServo");
        leverServo = hardwareMap.get(Servo.class,"leverServo");
        intakeMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        intakeMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rotationMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rotationMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
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
        /*
        if (timer.milliseconds() <= 140) {
            sorterServo.setPower(1);
        } else {
            sorterServo.setPower(0);
        }
        telemetry.addData("encoder ticks", intakeMotor.getCurrentPosition());

         */

        /*
        if (gamepad1.x && !sorting) {
            sorting = true;
            timer.reset();
        }

        if (sorting) {
            // 🔧 about 0.115s ≈ 60° on Axon Max at ~6V
            if (timer.milliseconds() <= 70) {
                sorterServo.setPower(1.0);   // full speed
            } else {
                sorterServo.setPower(0.0);   // stop
                sorting = false;             // done with this step
            }
        } else {
            sorterServo.setPower(0.0);       // idle
        }

        telemetry.addData("sorting", sorting);
        telemetry.addData("timer (ms)", timer.milliseconds());
        telemetry.update();

         */

        boolean xPressed = gamepad1.x;
        boolean xJustPressed = xPressed && !prevX;

        if (xJustPressed && !sorting) {
            sorting = true;
            timer.reset();
        }

// Run the 70 ms action
        if (sorting) {
            if (timer.milliseconds() <= timeAmount) {
                sorterServo.setPower(1.0);
            } else {
                sorterServo.setPower(0.0);
                sorting = false;
            }
        } else {
            sorterServo.setPower(0.0);
        }

// Save button state for next loop
        prevX = xPressed;

        telemetry.addData("sorting", sorting);
        telemetry.addData("timer", timer.milliseconds());
        telemetry.addData("xJustPressed", xJustPressed);
        telemetry.update();


    }
}