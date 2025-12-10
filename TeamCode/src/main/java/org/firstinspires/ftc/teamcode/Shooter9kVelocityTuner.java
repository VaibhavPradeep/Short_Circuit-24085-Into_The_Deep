package org.firstinspires.ftc.teamcode;

import static com.qualcomm.robotcore.hardware.DcMotor.RunMode.RUN_USING_ENCODER;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;

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

// jiojoi
@Config
@TeleOp(name = "Shooter9kVelocityTuner")
public class Shooter9kVelocityTuner extends OpMode {

    // Shooter motor (with encder)===
    private DcMotorEx shootingMotor;

    // Shooter servo
    private Servo pitchServo;

    // === Dashboard Tunables ===

    // --- MODE SELECT ---
    public static boolean useVelocityControl = true;

    // --- Power Mode ---
    public static double shooterPower = 0.0;

    // --- Velocity Mode ---
    public static double targetWheelRpm = 5700.0;
    public static int motorTicksPerRev = 28;
    public static double gearRatioWheelOverMotor = 1.5;

    // PID gains
    public static double kPs = 0.01;
    public static double kIs = 0.0;
    public static double kDs = 0.0;
    public static double kFs = 0.0;

    public static double rpmTolerance = 100.0;

    public static double servoPosition = 0.0;
    public static double leverPos = 0.0;

    private PIDController shooterPid;

    private ElapsedTime velTimer = new ElapsedTime();
    private int lastPosition = 0;

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
    public static double pitchPos = 0;

    public static int encoderAmount = 89;

    boolean prevPressed = false;

    private long lastAdvanceTime = 0;
    private static final long ADVANCE_DEBOUNCE_MS = 200;

    DcMotor intakeMotor;
    DcMotor rotationMotor;

    private DcMotor sorterMotor;
    private PIDController pid;

    final int READ_PERIOD = 1;

    private static final double TICKS_PER_REV = 537.6;

    private double[] positionDegrees = { 0, 60, 120, 180, 240, 300 };

    private int currentIndex = 0;
    private int targetTicks = 0;


    @Override
    public void init() {

        sorterMotor = hardwareMap.get(DcMotor.class, "sorterMotor");
        sorterMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        sorterMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        sorterMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        pid = new PIDController(kPs, kIs, kDs);
        pid.setSetPoint(targetTicks);

        intakeMotor = hardwareMap.get(DcMotor.class, "intakeMotor");
        rotationMotor = hardwareMap.get(DcMotor.class, "rotationMotor");
        huskyLens = hardwareMap.get(HuskyLens.class, "huskylens");
        huskyLens2 = hardwareMap.get(HuskyLens.class, "huskylens2");
        sorterMotor = hardwareMap.get(DcMotor.class, "sorterMotor");
        leverServo = hardwareMap.get(Servo.class,"leverServo");
        turretImu = hardwareMap.get(IMU.class, "turretImu");

        RevHubOrientationOnRobot orientationOnRobot = new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.UP,
                RevHubOrientationOnRobot.UsbFacingDirection.FORWARD
        );

        turretImu.initialize(new IMU.Parameters(orientationOnRobot));

        rotationMotor.setMode(RUN_USING_ENCODER);
        rotationMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        rateLimit = new Deadline(READ_PERIOD, TimeUnit.SECONDS);
        rateLimit.expire();
        leverServo.setPosition(0);

        huskyLens.selectAlgorithm(HuskyLens.Algorithm.TAG_RECOGNITION);

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

        sorterMotor.setMode(RUN_USING_ENCODER);
        sorterMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        shootingMotor = hardwareMap.get(DcMotorEx.class, "shootingMotor");
        pitchServo = hardwareMap.get(Servo.class, "pitchServo");

        shootingMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        shootingMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        shootingMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        shooterPid = new PIDController(kPs, kIs, kDs);

        pitchServo.setPosition(servoPosition);

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
    }


    @Override
    public void loop() {

        pitchServo.setPosition(servoPosition);
        leverServo.setPosition(leverPos);

        if (!useVelocityControl) {

            shootingMotor.setPower(shooterPower);

            telemetry.addLine("=== Shooter Power Mode ===");
            telemetry.addData("Power", shooterPower);
            telemetry.update();
            return;
        }

        // === VELOCITY MODE ===

        // Convert wheel RPM target → motor RPM
        double targetMotorRpm = targetWheelRpm / gearRatioWheelOverMotor;

        // Encoder velocity (ticks/s)
        double actualMotorTicksPerSec = shootingMotor.getVelocity();

        // Motor RPM
        double actualMotorRpm = (actualMotorTicksPerSec * 60.0) / motorTicksPerRev;

        // Wheel RPM
        double actualWheelRpm = actualMotorRpm * gearRatioWheelOverMotor;

        // Error
        double errorWheelRpm = targetWheelRpm - actualWheelRpm;

        shooterPid.setPID(kPs, kIs, kDs);

        double pidOutput = shooterPid.calculate(actualWheelRpm, targetWheelRpm);

        double ff = kFs * targetWheelRpm;

        double power = pidOutput + ff;
        power = Math.max(0.0, Math.min(1.0, power));

        // 0.01 KP AND 5700 TARGET RPM

        shootingMotor.setPower(power);

        boolean atSpeed = Math.abs(errorWheelRpm) <= rpmTolerance;

        telemetry.addLine("=== Shooter Velocity PID Mode ===");
        telemetry.addData("Target wheel RPM", targetWheelRpm);
        telemetry.addData("Actual wheel RPM", actualWheelRpm);
        telemetry.addData("Error", errorWheelRpm);
        telemetry.addData("At Speed?", atSpeed);
        telemetry.addData("Power", power);
        telemetry.update();
    }


    @Override
    public void stop() {
        shootingMotor.setPower(0);
    }
}
