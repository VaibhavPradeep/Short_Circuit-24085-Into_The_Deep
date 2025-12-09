package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.controller.PIDController;
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
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import java.util.concurrent.TimeUnit;

@Config
@TeleOp(name = "use this teleop")
public class TeleOp18 extends OpMode {
    // lever pos 0 as up and 0.123 and the bottom
    // pitch innit pos should be 0.5
    final int READ_PERIOD = 1;
    // TODO: find lever pos

    private PIDController controller;
    public static double p = -0.003;
    public static double i = 0;
    public static double d = -0.1;
    DcMotor intakeMotor;
    Servo pitchServo;
    DcMotor rotationMotor;
    DcMotor shootingMotor;
    DcMotor sorterMotor;
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
    public static double pitchPos = 0;

    public static double encoderAmount;

    boolean prevPressed = false;

    public void driveMecanum(double left_y, double left_x, double right_x){
        double maxPower = Math.max(Math.abs(left_y) + Math.abs(left_x) + Math.abs(right_x), 1);
        frontLeft.setPower((left_y - left_x - right_x) / maxPower);
        frontRight.setPower((left_y + left_x + right_x) / maxPower);
        backLeft.setPower((left_y + left_x - right_x) / maxPower);
        backRight.setPower((left_y - left_x + right_x) / maxPower);
    }

    /* public void dPadMove(String direction) {
        int pos = rotationMotor.getCurrentPosition();
        if(direction.equals("up")) {
            pos += 75;
        } else if(direction.equals("down") && pos > 75) {
            pos -= 75;
        }
        rotationMotor.setTargetPosition(pos);
        rotationMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rotationMotor.setPower(0.75);
        if (rotationMotor.getCurrentPosition() == pos) {
            rotationMotor.setPower(0);
        }
    } */

    @Override
    public void init() {
        controller = new PIDController(p,i,d);
        intakeMotor = hardwareMap.get(DcMotor.class, "intakeMotor");
        pitchServo = hardwareMap.get(Servo.class,"pitchServo");
        rotationMotor = hardwareMap.get(DcMotor.class, "rotationMotor");
        shootingMotor = hardwareMap.get(DcMotor.class, "shootingMotor");
        huskyLens = hardwareMap.get(HuskyLens.class, "huskylens");
        huskyLens2 = hardwareMap.get(HuskyLens.class, "huskylens2");
        sorterMotor = hardwareMap.get(DcMotor.class, "sorterMotor");
        leverServo = hardwareMap.get(Servo.class,"leverServo");
        turretImu = hardwareMap.get(IMU.class, "turretImu");


        // Set up parameters for turret orientation (adjust based on mounting)
        RevHubOrientationOnRobot orientationOnRobot = new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.UP,
                RevHubOrientationOnRobot.UsbFacingDirection.FORWARD
        );

        // Initialize
        turretImu.initialize(new IMU.Parameters(orientationOnRobot));

        rotationMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rotationMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        // Rate limiter for telemetry
        rateLimit = new Deadline(READ_PERIOD, TimeUnit.SECONDS);
        rateLimit.expire();
        leverServo.setPosition(0);
        // Choose the algorithm
        huskyLens.selectAlgorithm(HuskyLens.Algorithm.TAG_RECOGNITION);
        // huskyLens.selectAlgorithm(HuskyLens.Algorithm.COLOR_RECOGNITION);
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
    }

    public void start() {
        timer.reset();
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
        if (gamepad1.x) {
            intakeMotor.setPower(-1);
        }

        if (gamepad2.x) {
            shootingMotor.setPower(1);
        }
        if (gamepad2.y) {
            shootingMotor.setPower(0);
        }

        if (gamepad2.dpad_up) {
            leverServo.setPosition(0.2);
        }
        if (gamepad2.dpad_down) {
            leverServo.setPosition(0);
        }

        boolean aPressed = gamepad1.a;


        controller.setPID(p,i,d);

        int pos = sorterMotor.getCurrentPosition();

        if (aPressed && !prevPressed) {
            pos += encoderAmount;
            /*
            sorterMotor.setTargetPosition(pos);
            sorterMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            sorterMotor.setPower(0.75);
            if (sorterMotor.getCurrentPosition() == pos) {
                sorterMotor.setPower(0);
            }

             */
        }


        double pidOutput = controller.calculate(pos, sorterMotor.getCurrentPosition());

        prevPressed = aPressed;

        sorterMotor.setPower(pidOutput);

        
        /*
        controller.setPID(p,i,d);

        double pidOutput = controller.calculate(target, sorterMotor.getCurrentPosition());
        sorterMotor.setPower(pidOutput);
         */
    }
}