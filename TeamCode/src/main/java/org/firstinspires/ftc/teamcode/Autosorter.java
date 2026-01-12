package org.firstinspires.ftc.teamcode;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.hardware.dfrobot.HuskyLens;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcore.internal.system.Deadline;
import java.util.concurrent.TimeUnit;

@TeleOp(name = "autosorter")
public class Autosorter extends LinearOpMode {
    public double fShooting = 0;
    public double pShooting = 0;
    public double curTargetVelocity = 0;
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
    private static final double NO_DETECTION_SKIP_TIMEOUT_MS = 7000; // 7 seconds → skip 120° if no color
    private static final double SINGLE_ADVANCE_TIMEOUT_MS = 5000; // NEW: 5s max per 60° step to prevent infinite spin
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
    public enum MotifCode {
        NONE,
        TAG_ID_1_GPP,
        TAG_ID_2_PGP,
        TAG_ID_3_PPG
    }
    enum Motif1GPP { G, P1, P2 }
    enum Motif2PGP { P1, G, P2 }
    enum Motif3PPG { P1, P2, G }
    MotifCode motifCode = MotifCode.TAG_ID_1_GPP;
    Motif1GPP motif1 = Motif1GPP.G;
    Motif2PGP motif2 = Motif2PGP.P1;
    Motif3PPG motif3 = Motif3PPG.P1;

    private enum SorterState {
        IDLE,
        EXPECT_GREEN,
        WAIT_FOR_GREEN_STABLE,
        REV_UP_GREEN,
        LEVER_UP_GREEN,
        LEVER_DOWN_GREEN,
        ADVANCE_AFTER_GREEN,
        EXPECT_PURPLE1,
        WAIT_FOR_PURPLE1_STABLE,
        REV_UP_PURPLE1,
        LEVER_UP_PURPLE1,
        LEVER_DOWN_PURPLE1,
        ADVANCE_AFTER_PURPLE1,
        EXPECT_PURPLE2,
        WAIT_FOR_PURPLE2_STABLE,
        REV_UP_PURPLE2,
        LEVER_UP_PURPLE2,
        LEVER_DOWN_PURPLE2,
        ADVANCE_AFTER_PURPLE2,
        ERROR
    }
    private SorterState currentState = SorterState.IDLE;
    private ElapsedTime stateTimer = new ElapsedTime();
    private static final double DETECTION_DEBOUNCE_MS = 200;
    private static final double LEVER_UP_MS = 500; // 0.5 seconds
    private static final double REV_UP_GREEN_MS = 3000;
    private static final double REV_UP_PURPLE_MS = 1000;
    private static final double STATE_TIMEOUT_MS = 10000; // safety
    private int sorterPressesRemaining = 0;
    private boolean sorterAdvanceInProgress = false;
    private boolean sorterAdvanceWaitingForDebounce = false;
    private long nextAdvanceStartTime = 0;
    private ElapsedTime advanceTimer = new ElapsedTime(); // NEW: for per-advance timeout

    @Override
    public void runOpMode() throws InterruptedException {
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
        leverServo = hardwareMap.get(Servo.class,"leverServo");
        turretImu = hardwareMap.get(IMU.class, "turretImu");
        RevHubOrientationOnRobot orientationOnRobot =
                new RevHubOrientationOnRobot(
                        RevHubOrientationOnRobot.LogoFacingDirection.UP,
                        RevHubOrientationOnRobot.UsbFacingDirection.FORWARD
                );
        turretImu.initialize(new IMU.Parameters(orientationOnRobot));
        pitchServo.setDirection(Servo.Direction.REVERSE);
        pitchServo.setPosition(0);
        shootingMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        shootingMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        shootingMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        PIDFCoefficients pidfShooting =
                new PIDFCoefficients(pShooting, 0, 0, fShooting);
        shootingMotor.setPIDFCoefficients(
                DcMotor.RunMode.RUN_USING_ENCODER, pidfShooting
        );
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
        telemetry.addData("status", "initialized");
        telemetry.update();
        waitForStart();
        if (isStopRequested()) return;
        pitchServo.setPosition(0.42);
        currentState = SorterState.EXPECT_GREEN;
        stateTimer.reset();

        while (opModeIsActive() && !isStopRequested()) {
            shootingMotor.setVelocity(curTargetVelocity);
            updateFSM();
            idle();
        }
    }

    private void updateFSM() {
        telemetry.addData("State", currentState);
        telemetry.addData("Motif", motif1);

        switch (currentState) {
            case IDLE:
                break;

            case EXPECT_GREEN:
                if (detectGreen()) {
                    transitionTo(SorterState.WAIT_FOR_GREEN_STABLE);
                } else if (stateTimer.milliseconds() >= NO_DETECTION_SKIP_TIMEOUT_MS) {
                    startSorterAdvance(2); // skip 120°
                    stateTimer.reset();
                    telemetry.addData("SKIP", "No green - advanced 120°");
                }
                break;

            case WAIT_FOR_GREEN_STABLE:
                if (stateTimer.milliseconds() >= DETECTION_DEBOUNCE_MS) {
                    if (detectGreen()) {
                        transitionTo(SorterState.REV_UP_GREEN);
                    } else {
                        transitionTo(SorterState.EXPECT_GREEN);
                    }
                }
                break;

            case REV_UP_GREEN:
                curTargetVelocity = 1650;
                if (stateTimer.milliseconds() >= REV_UP_GREEN_MS) {
                    transitionTo(SorterState.LEVER_UP_GREEN);
                }
                checkTimeout();
                break;

            case LEVER_UP_GREEN:
                leverUp();
                if (stateTimer.milliseconds() >= LEVER_UP_MS) {
                    transitionTo(SorterState.LEVER_DOWN_GREEN);
                }
                checkTimeout();
                break;

            case LEVER_DOWN_GREEN:
                leverDown();
                startSorterAdvance(2);
                transitionTo(SorterState.ADVANCE_AFTER_GREEN);
                break;

            case ADVANCE_AFTER_GREEN:
                updateSorterAdvance();
                if (sorterPressesRemaining == 0 && !sorterAdvanceInProgress && !sorterAdvanceWaitingForDebounce) {
                    motif1 = Motif1GPP.P1;
                    transitionTo(SorterState.EXPECT_PURPLE1);
                }
                checkTimeout();
                break;

            case EXPECT_PURPLE1:
                if (detectPurple()) {
                    transitionTo(SorterState.WAIT_FOR_PURPLE1_STABLE);
                } else if (stateTimer.milliseconds() >= NO_DETECTION_SKIP_TIMEOUT_MS) {
                    startSorterAdvance(2);
                    stateTimer.reset();
                    telemetry.addData("SKIP", "No purple1 - advanced 120°");
                }
                break;

            case WAIT_FOR_PURPLE1_STABLE:
                if (stateTimer.milliseconds() >= DETECTION_DEBOUNCE_MS) {
                    if (detectPurple()) {
                        transitionTo(SorterState.REV_UP_PURPLE1);
                    } else {
                        transitionTo(SorterState.EXPECT_PURPLE1);
                    }
                }
                break;

            case REV_UP_PURPLE1:
                if (curTargetVelocity == 0) curTargetVelocity = 1650;
                if (stateTimer.milliseconds() >= REV_UP_PURPLE_MS) {
                    transitionTo(SorterState.LEVER_UP_PURPLE1);
                }
                checkTimeout();
                break;

            case LEVER_UP_PURPLE1:
                leverUp();
                if (stateTimer.milliseconds() >= LEVER_UP_MS) {
                    transitionTo(SorterState.LEVER_DOWN_PURPLE1);
                }
                checkTimeout();
                break;

            case LEVER_DOWN_PURPLE1:
                leverDown();
                startSorterAdvance(2);
                transitionTo(SorterState.ADVANCE_AFTER_PURPLE1);
                break;

            case ADVANCE_AFTER_PURPLE1:
                updateSorterAdvance();
                if (sorterPressesRemaining == 0 && !sorterAdvanceInProgress && !sorterAdvanceWaitingForDebounce) {
                    motif1 = Motif1GPP.P2;
                    transitionTo(SorterState.EXPECT_PURPLE2);
                }
                checkTimeout();
                break;

            case EXPECT_PURPLE2:
                if (detectPurple()) {
                    transitionTo(SorterState.WAIT_FOR_PURPLE2_STABLE);
                } else if (stateTimer.milliseconds() >= NO_DETECTION_SKIP_TIMEOUT_MS) {
                    startSorterAdvance(2);
                    stateTimer.reset();
                    telemetry.addData("SKIP", "No purple2 - advanced 120°");
                }
                break;

            case WAIT_FOR_PURPLE2_STABLE:
                if (stateTimer.milliseconds() >= DETECTION_DEBOUNCE_MS) {
                    if (detectPurple()) {
                        transitionTo(SorterState.REV_UP_PURPLE2);
                    } else {
                        transitionTo(SorterState.EXPECT_PURPLE2);
                    }
                }
                break;

            case REV_UP_PURPLE2:
                if (curTargetVelocity == 0) curTargetVelocity = 1650;
                if (stateTimer.milliseconds() >= REV_UP_PURPLE_MS) {
                    transitionTo(SorterState.LEVER_UP_PURPLE2);
                }
                checkTimeout();
                break;

            case LEVER_UP_PURPLE2:
                leverUp();
                if (stateTimer.milliseconds() >= LEVER_UP_MS) {
                    transitionTo(SorterState.LEVER_DOWN_PURPLE2);
                }
                checkTimeout();
                break;

            case LEVER_DOWN_PURPLE2:
                leverDown();
                startSorterAdvance(2);
                transitionTo(SorterState.ADVANCE_AFTER_PURPLE2);
                break;

            case ADVANCE_AFTER_PURPLE2:
                updateSorterAdvance();
                if (sorterPressesRemaining == 0 && !sorterAdvanceInProgress && !sorterAdvanceWaitingForDebounce) {
                    motif1 = Motif1GPP.G;
                    transitionTo(SorterState.EXPECT_GREEN);
                }
                checkTimeout();
                break;

            case ERROR:
                sorterMotor.setPower(0);
                shootingMotor.setVelocity(0);
                telemetry.addData("ERROR", "Timeout or stuck");
                break;
        }
    }

    private void transitionTo(SorterState newState) {
        currentState = newState;
        stateTimer.reset();
    }

    private void checkTimeout() {
        if (stateTimer.milliseconds() > STATE_TIMEOUT_MS) {
            transitionTo(SorterState.ERROR);
        }
    }

    private void startSorterAdvance(int presses) {
        sorterPressesRemaining = presses;
        sorterAdvanceInProgress = false;
        sorterAdvanceWaitingForDebounce = false;
        nextAdvanceStartTime = 0;
        updateSorterAdvance(); // Try to start immediately
    }

    private void updateSorterAdvance() {
        if (sorterAdvanceWaitingForDebounce) {
            if (System.currentTimeMillis() >= nextAdvanceStartTime) {
                sorterAdvanceWaitingForDebounce = false;
                sorterAdvanceInProgress = true;
                advanceTimer.reset(); // NEW: reset per-step timer
            }
            return;
        }

        if (!sorterAdvanceInProgress && sorterPressesRemaining > 0) {
            long now = System.currentTimeMillis();
            if (now - lastAdvanceTime < ADVANCE_DEBOUNCE_MS) {
                sorterAdvanceWaitingForDebounce = true;
                nextAdvanceStartTime = lastAdvanceTime + ADVANCE_DEBOUNCE_MS;
                return;
            }
            currentIndex = (currentIndex + 1) % positionDegrees.length;
            targetTicks = degreesToTicks(positionDegrees[currentIndex]);
            pid.setSetPoint(targetTicks);
            sorterAdvanceInProgress = true;
            advanceTimer.reset(); // NEW: start per-step timer
        }

        if (sorterAdvanceInProgress) {
            // NEW: Check per-advance timeout first
            if (advanceTimer.milliseconds() > SINGLE_ADVANCE_TIMEOUT_MS) {
                telemetry.addData("ERROR", "Single advance timeout - stopping spin");
                sorterMotor.setPower(0);
                sorterAdvanceInProgress = false;
                sorterPressesRemaining = 0; // Abort remaining presses
                return;
            }

            double currentPos = sorterMotor.getCurrentPosition();
            double error = targetTicks - currentPos;
            double power = pid.calculate(currentPos) + kS * Math.signum(error);
            if (Math.abs(error) < 5) {
                sorterMotor.setPower(0);
                sorterAdvanceInProgress = false;
                sorterPressesRemaining--;
                lastAdvanceTime = System.currentTimeMillis();
                if (sorterPressesRemaining > 0) {
                    sorterAdvanceWaitingForDebounce = true;
                    nextAdvanceStartTime = lastAdvanceTime + ADVANCE_DEBOUNCE_MS;
                }
            } else {
                power = Math.max(-0.6, Math.min(0.6, power));
                sorterMotor.setPower(power);
            }
        }
    }

    public boolean detectGreen() {
        return colorSensor.green() >= 300;
    }

    public boolean detectPurple() {
        return colorSensor.red() >= 150 && colorSensor.red() <= 250 &&
                colorSensor.blue() >= 300 && colorSensor.blue() <= 450 &&
                colorSensor.green() >= 200 && colorSensor.green() <= 300;
    }

    public void leverDown() { leverServo.setPosition(0); }
    public void leverUp() { leverServo.setPosition(0.2); }

    private int degreesToTicks(double deg) {
        return (int)((deg / 360.0) * TICKS_PER_REV);
    }
}