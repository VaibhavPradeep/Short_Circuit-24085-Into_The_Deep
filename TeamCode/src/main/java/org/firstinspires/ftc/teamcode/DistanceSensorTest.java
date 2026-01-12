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
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.internal.system.Deadline;

import java.util.ArrayList;
import java.util.List;
import java.util.concurrent.TimeUnit;

@Config
@TeleOp(name = "ColorSensorTest")
public class DistanceSensorTest extends OpMode {
    public static double WAIT_TIME = 0;


    public static double targetPower = 0;
    public static double LONG_SHOT = 0.87;
    public static double SHORT_SHOT = 0.795;
    public static double LONG_SERVO_POS = 0.3;
    public static double SHORT_SERVO_POS = 0.4;

    private DcMotor sorterMotor;
    private PIDController pid;

    final int READ_PERIOD = 1;
    private static final double TICKS_PER_REV = 537.6;

    private double[] positionDegrees = { 0, 60, 120, 180, 240, 300 };

    private int currentIndex = 0;
    private int targetTicks = 0;

    private double kP = 0.0029;
    private double kI = 0.0;
    private double kD = 0.00017;

    public static double kS = 0.02;

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

    DistanceSensor distanceSensor;
    IMU turretImu;

    boolean prevX = false;
    boolean sorting = false;

    public static int timeAmount = 110;
    ElapsedTime timer = new ElapsedTime();
    public static double leverPos = 0;
    public static double pitchPos = 0.42;

    public static int encoderAmount = 89;



    boolean waiting = false;
    double waitDuration = 0;

    @Override
    public void init() {
        sorterMotor = hardwareMap.get(DcMotor.class, "sorterMotor");
        sorterMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        sorterMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        sorterMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        pid = new PIDController(kP, kI, kD);

        targetTicks = degreesToTicks(positionDegrees[currentIndex]);
        pid.setSetPoint(targetTicks);

        intakeMotor = hardwareMap.get(DcMotor.class, "intakeMotor");
        pitchServo = hardwareMap.get(Servo.class,"pitchServo");
        rotationMotor = hardwareMap.get(DcMotor.class, "rotationMotor");
        shootingMotor = hardwareMap.get(DcMotorEx.class, "shootingMotor");
        huskyLens = hardwareMap.get(HuskyLens.class, "huskylens");
        huskyLens2 = hardwareMap.get(HuskyLens.class, "huskylens2");
        distanceSensor = hardwareMap.get(DistanceSensor.class, "distanceSensor");
        leverServo = hardwareMap.get(Servo.class,"leverServo");
        turretImu = hardwareMap.get(IMU.class, "turretImu");

        RevHubOrientationOnRobot orientationOnRobot = new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.UP,
                RevHubOrientationOnRobot.UsbFacingDirection.FORWARD
        );

        turretImu.initialize(new IMU.Parameters(orientationOnRobot));

        pitchServo.setDirection(Servo.Direction.REVERSE);
        pitchServo.setPosition(0);

        shootingMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        shootingMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        shootingMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        rotationMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rotationMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        rateLimit = new Deadline(READ_PERIOD, TimeUnit.SECONDS);
        rateLimit.expire();
        leverServo.setPosition(0);

        huskyLens.selectAlgorithm(HuskyLens.Algorithm.TAG_RECOGNITION);

        frontLeft = hardwareMap.get(DcMotor.class, "frontLeft");
        frontRight = hardwareMap.get(DcMotor.class, "frontRight");
        backLeft = hardwareMap.get(DcMotor.class, "backLeft");
        backRight = hardwareMap.get(DcMotor.class, "backRight");
        colorSensor = hardwareMap.get(ColorSensor.class, "colorSensor");

        frontRight.setDirection(DcMotorSimple.Direction.REVERSE);
        backRight.setDirection(DcMotorSimple.Direction.REVERSE);
        intakeMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        frontLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        frontRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        telemetry.addData("status", "initialized");
    }

    @Override
    public void loop() {
            
    }

    public void waitMs(double milliseconds) {
        timer.reset();
        while (timer.milliseconds() < milliseconds) {

        }
    }


    private int degreesToTicks(double deg) {
        return (int)((deg / 360.0) * TICKS_PER_REV);
    }
}