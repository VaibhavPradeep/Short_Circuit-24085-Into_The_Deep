package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.hardware.dfrobot.HuskyLens;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.internal.system.Deadline;

import java.util.concurrent.TimeUnit;

public class IntakeLiftCamera {
    
    DcMotor sorterMotor;
    DcMotor intakeMotor;
    DcMotorEx shootingMotor;
    Servo leverServo;
    DcMotor rotationMotor;
    Servo pitchServo;
    HuskyLens huskyLens;
    HuskyLens huskyLens2;

    IMU turretImu;

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
    
    public void initILC(HardwareMap hwMap) {
        sorterMotor = hwMap.get(DcMotor.class, "sorterMotor");
        sorterMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        sorterMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        sorterMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        pid = new PIDController(kP, kI, kD);

        // initial target
        targetTicks = degreesToTicks(positionDegrees[currentIndex]);
        pid.setSetPoint(targetTicks);

        intakeMotor = hwMap.get(DcMotor.class, "intakeMotor");
        pitchServo = hwMap.get(Servo.class,"pitchServo");
        rotationMotor = hwMap.get(DcMotor.class, "rotationMotor");
        shootingMotor = hwMap.get(DcMotorEx.class, "shootingMotor");
        huskyLens = hwMap.get(HuskyLens.class, "huskylens");
        huskyLens2 = hwMap.get(HuskyLens.class, "huskylens2");
        leverServo = hwMap.get(Servo.class,"leverServo");
        turretImu = hwMap.get(IMU.class, "turretImu");


        // Set up parameters for turret orientation (adjust based on mounting)
        RevHubOrientationOnRobot orientationOnRobot = new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.UP,
                RevHubOrientationOnRobot.UsbFacingDirection.FORWARD
        );

        // Initialize
        turretImu.initialize(new IMU.Parameters(orientationOnRobot));


        pitchServo.setDirection(Servo.Direction.REVERSE);
        pitchServo.setPosition(0);

        shootingMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        shootingMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        shootingMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        // shooter pid end

        rotationMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rotationMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        // Rate limiter for telemetry
        leverServo.setPosition(0);
        // Choose the algorithm
        huskyLens.selectAlgorithm(HuskyLens.Algorithm.TAG_RECOGNITION);

        intakeMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        pitchServo.setDirection(Servo.Direction.REVERSE);
        pitchServo.setPosition(0);
    }

    public void leverUp() {
        leverServo.setPosition(0.2);
    }
    public void leverDown() {
        leverServo.setPosition(0);
    }

    public void intakeOn() {
        intakeMotor.setPower(1);
    }

    public void intakeOff() {
        intakeMotor.setPower(0);
    }

    public void pitch() {
        pitchServo.setPosition(0.42);
    }

    public void longShoot() {
        shootingMotor.setPower(0.875);
    }

    public void shortShoot() {
        shootingMotor.setPower(0.795);
    }

    public void shootingOff() {
        shootingMotor.setPower(0);
    }

    public void sorting(int rotations) {
    }

    private int degreesToTicks(double deg) {
        return (int)((deg / 360.0) * TICKS_PER_REV);
    }
}
