package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.hardware.dfrobot.HuskyLens;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.internal.system.Deadline;

import java.util.concurrent.TimeUnit;

@Config
@TeleOp(name = "REAL NEW TELEOP")
public class TeleOpNew extends OpMode {

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
    private double kP = 0.003;
    private double kI = 0.0;
    private double kD = 0.00011;

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

    public void driveMecanum(double left_y, double left_x, double right_x){
        double maxPower = Math.max(Math.abs(left_y) + Math.abs(left_x) + Math.abs(right_x), 1);
        frontLeft.setPower((left_y - left_x - right_x) / maxPower);
        frontRight.setPower((left_y + left_x + right_x) / maxPower);
        backLeft.setPower((left_y + left_x - right_x) / maxPower);
        backRight.setPower((left_y - left_x + right_x) / maxPower);
    }

    public void driveMecanumSlower(double left_y, double left_x, double right_x){
        double maxPower = Math.max(Math.abs(left_y) + Math.abs(left_x) + Math.abs(right_x), 1);
        frontLeft.setPower(((left_y - left_x - right_x) / maxPower) / 3);
        frontRight.setPower(((left_y + left_x + right_x) / maxPower) / 3);
        backLeft.setPower(((left_y + left_x - right_x) / maxPower) / 3);
        backRight.setPower(((left_y - left_x + right_x) / maxPower) / 3);
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
        sorterMotor = hardwareMap.get(DcMotor.class, "sorterMotor");
        sorterMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        sorterMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        sorterMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        pid = new PIDController(kP, kI, kD);

        // initial target
        targetTicks = degreesToTicks(positionDegrees[currentIndex]);
        pid.setSetPoint(targetTicks);

        intakeMotor = hardwareMap.get(DcMotor.class, "intakeMotor");
        pitchServo = hardwareMap.get(Servo.class,"pitchServo");
        rotationMotor = hardwareMap.get(DcMotor.class, "rotationMotor");
        shootingMotor = hardwareMap.get(DcMotorEx.class, "shootingMotor");
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

        // shooter pid



        pitchServo.setDirection(Servo.Direction.REVERSE);
        pitchServo.setPosition(0);

        shootingMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        shootingMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        shootingMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        // shooter pid end

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
        backRight.setDirection(DcMotorSimple.Direction.FORWARD);
        frontLeft.setDirection(DcMotorSimple.Direction.FORWARD);
        backLeft.setDirection(DcMotorSimple.Direction.FORWARD);
        intakeMotor.setDirection(DcMotorSimple.Direction.REVERSE);
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
        pitchServo.setDirection(Servo.Direction.REVERSE);
        pitchServo.setPosition(0);

        sorterMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        sorterMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        telemetry.addData("status", "initialized");
    }

    public void start() {
        timer.reset();
        pitchServo.setPosition(0.42);
    }

    @Override
    public void loop() {

        pitchServo.setPosition(0.42);

        if (gamepad1.left_trigger > 0.75) {
            driveMecanumSlower(gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x);
        } else {
            driveMecanum(gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x);
        }

        if (gamepad1.a) {
            intakeMotor.setPower(1);
        }
        if (gamepad1.b) {
            intakeMotor.setPower(0);
        }
        if (gamepad1.x) {
            intakeMotor.setPower(-1);
        }

        if (gamepad2.dpad_up) {
            leverServo.setPosition(0.2);
        }
        else {
            leverServo.setPosition(0);
        }


        //boolean aPressed = gamepad2.a;
        //controller.setPID(p,i,d);

//        if (aPressed && !prevPressed) {
//            sorterTarget += encoderAmount;
//
//            sorterMotor.setTargetPosition(pos);
//            sorterMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//            sorterMotor.setPower(0.75);
//            if (sorterMotor.getCurrentPosition() == pos) {
//                sorterMotor.setPower(0);
//            }
//        }
//
//        prevPressed = aPressed;

        //double pidOutput = controller.calculate(sorterMotor.getCurrentPosition(), sorterTarget);

        //sorterMotor.setPower(pidOutput);



        /*
        boolean aPressed = gamepad2.a;


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
         */


        long now = System.currentTimeMillis();

        // One-button advance (with debounce)
        if (gamepad2.a && (now - lastAdvanceTime) > ADVANCE_DEBOUNCE_MS) {
            currentIndex++;
            if (currentIndex >= positionDegrees.length) {
                currentIndex = 0;
            }
            targetTicks = degreesToTicks(positionDegrees[currentIndex]);
            pid.setSetPoint(targetTicks);

            lastAdvanceTime = now;
        }

        // Read current position
        double currentPos = sorterMotor.getCurrentPosition();

        // PID controller: compute power
        double power = pid.calculate(currentPos);

        // Deadband / tolerance (to avoid jitter near target)
        if (Math.abs(targetTicks - currentPos) < 10) {
            power = 0;
        }

        // Clamp to safe motor power
        power = Math.max(-0.6, Math.min(0.6, power));

        sorterMotor.setPower(power);

        // Telemetry for tuning/debugging
        telemetry.addData("Cur ticks", currentPos);
        telemetry.addData("Tgt ticks", targetTicks);
        telemetry.addData("Index", currentIndex);
        telemetry.addData("Error", targetTicks - currentPos);
        telemetry.addData("Power", power);
        telemetry.update();


        // 56.4
        //43.2
        // SHOOTER
        boolean shooterEnabled1 = gamepad2.left_trigger > 0.75;
        boolean shooterEnabled2 = gamepad2.right_trigger > 0.75;

        if (!shooterEnabled2 && !shooterEnabled1) {
            targetPower = 0;
        } else if (shooterEnabled2 && !shooterEnabled1){
            targetPower = LONG_SHOT;
        } else if (!shooterEnabled2 && shooterEnabled1) {
            targetPower = SHORT_SHOT;
        } else if (shooterEnabled2 && shooterEnabled1) {
            targetPower = 0;
        }

        pitchServo.setPosition(pitchPos);
        shootingMotor.setPower(targetPower);

    }

    @Override
    public void stop() {
        sorterMotor.setPower(0);
    }

    private int degreesToTicks(double deg) {
        return (int)((deg / 360.0) * TICKS_PER_REV);
    }
}
