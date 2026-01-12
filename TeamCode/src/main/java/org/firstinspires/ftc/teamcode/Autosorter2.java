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

@TeleOp(name = "autosorter2")
public class Autosorter2 extends LinearOpMode {

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

    // CHANGED: use local enums (previous bug) — unchanged semantics otherwise
    MotifCode motifCode = MotifCode.TAG_ID_1_GPP; // CHANGED
    Motif1GPP motif1 = Motif1GPP.G;               // CHANGED
    Motif2PGP motif2 = Motif2PGP.P1;              // CHANGED
    Motif3PPG motif3 = Motif3PPG.P1;              // CHANGED

    // CHANGED: FSM infrastructure
    private enum AutoState {
        DETECT_COLOR,
        SPIN_UP,
        FIRE_WAIT,
        ADVANCE_SORTER
    }
    private AutoState autoState = AutoState.DETECT_COLOR; // CHANGED
    private ElapsedTime stateTimer = new ElapsedTime();   // CHANGED
    private long spinUpDurationMs = 0;                    // CHANGED - set per-case
    // CHANGED: sorter non-blocking advance management
    private int sorterPressesRemaining = 0;               // CHANGED
    private boolean sorterAdvanceInProgress = false;      // CHANGED
    private boolean sorterAdvanceWaitingForDebounce = false; // CHANGED
    private long nextAdvanceStartTime = 0;                // CHANGED

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

        stateTimer.reset(); // CHANGED: make sure timer starts at 0

        // MAIN non-blocking loop — CHANGED: replaced single-check switch + blocking waits with FSM
        while (opModeIsActive() && !isStopRequested()) {

            // Always keep shooter velocity in sync with curTargetVelocity (same as before)
            shootingMotor.setVelocity(curTargetVelocity);

            // FSM behavior:
            switch (autoState) {

                case DETECT_COLOR:
                    // CHANGED: only transition when a detection AND sorter is NOT near forbidden angles
                    if (motif1 == Motif1GPP.G) {
                        if (detectGreen() && !isSorterNearForbiddenAngles(5.0)) {
                            // SETUP spin up for green case (3000 ms)
                            curTargetVelocity = 1650;
                            shootingMotor.setVelocity(curTargetVelocity); // CHANGED earlier: ensure velocity updated
                            spinUpDurationMs = 3000; // CHANGED: duration for G
                            stateTimer.reset(); // CHANGED
                            autoState = AutoState.SPIN_UP; // CHANGED
                        }
                    } else if (motif1 == Motif1GPP.P1 || motif1 == Motif1GPP.P2) {
                        if (detectPurple() && !isSorterNearForbiddenAngles(5.0)) {
                            // SETUP spin up for purple cases (1000 ms)
                            // (previous behavior had 1000 ms before leverUp)
                            spinUpDurationMs = 1000; // CHANGED: duration for P1/P2
                            // shooter velocity doesn't change in purple cases in original code, so keep as-is
                            stateTimer.reset(); // CHANGED
                            autoState = AutoState.SPIN_UP; // CHANGED
                        }
                    }
                    break;

                case SPIN_UP:
                    // CHANGED: wait (non-blocking) until spinUpDuration has elapsed
                    if (stateTimer.milliseconds() >= spinUpDurationMs) {
                        // perform fire action (lever up) and then proceed to FIRE_WAIT
                        leverUp();
                        stateTimer.reset();
                        autoState = AutoState.FIRE_WAIT;
                    }
                    break;

                case FIRE_WAIT:
                    // CHANGED: wait for the lever dwell time (250ms) non-blocking, then lower lever and start sorter advance
                    if (stateTimer.milliseconds() >= 250) {
                        leverDown();

                        // prepare to advance sorter by 2 for all cases (matching original behavior)
                        sorterPressesRemaining = 2;            // CHANGED: non-blocking multi-press
                        sorterAdvanceInProgress = false;      // will be started below
                        sorterAdvanceWaitingForDebounce = false;
                        nextAdvanceStartTime = 0;
                        // immediately attempt to start first advance step (startNextAdvance handles debounce)
                        startNextAdvance();                   // CHANGED: non-blocking start
                        autoState = AutoState.ADVANCE_SORTER;
                    }
                    break;

                case ADVANCE_SORTER:
                    // CHANGED: non-blocking advancement logic — step-by-step with same PID logic as before

                    // if waiting for debounce before starting the next step
                    if (sorterAdvanceWaitingForDebounce) {
                        if (System.currentTimeMillis() >= nextAdvanceStartTime) {
                            sorterAdvanceWaitingForDebounce = false;
                            // actually start movement now
                            sorterAdvanceInProgress = true;
                        } else {
                            // still waiting — do nothing this iteration
                            break;
                        }
                    }

                    if (!sorterAdvanceInProgress) {
                        // no movement currently in progress: if presses remain, try to start next press
                        if (sorterPressesRemaining > 0) {
                            startNextAdvance();
                        } else {
                            // finished all presses — transition back to DETECT and advance motif state like original code
                            // CHANGED: advance motif1 state exactly as original mapping
                            if (motif1 == Motif1GPP.G) {
                                motif1 = Motif1GPP.P1;
                            } else if (motif1 == Motif1GPP.P1) {
                                motif1 = Motif1GPP.P2;
                            } else if (motif1 == Motif1GPP.P2) {
                                motif1 = Motif1GPP.G;
                            }
                            autoState = AutoState.DETECT_COLOR;
                        }
                        break;
                    }

                    // If movement in progress, run PID update (same math as original code)
                    double currentPos = sorterMotor.getCurrentPosition();
                    double power = pid.calculate(currentPos);
                    double error = targetTicks - currentPos;
                    double staticFF = kS * Math.signum(error);
                    power += staticFF;

                    // Deadband / tolerance (to avoid jitter near target) — same tolerance as before (5)
                    if (Math.abs(error) < 5) {
                        sorterMotor.setPower(0);
                        sorterAdvanceInProgress = false; // finished this press
                        sorterPressesRemaining = Math.max(0, sorterPressesRemaining - 1);
                        // update lastAdvanceTime and set debounce before starting next
                        lastAdvanceTime = System.currentTimeMillis();
                        sorterAdvanceWaitingForDebounce = true;
                        nextAdvanceStartTime = lastAdvanceTime + ADVANCE_DEBOUNCE_MS;
                        break;
                    }

                    // Clamp to safe motor power — same clamp as before
                    power = Math.max(-0.6, Math.min(0.6, power));
                    sorterMotor.setPower(power);

                    break;
            }

            // let other subsystems run (non-blocking). previously you had idle() sometimes — keep it here
            idle();
        } // end while(opModeIsActive())
    } // end runOpMode()

    public boolean detectGreen() {
        return colorSensor.green() >= 300;
    }

    public boolean detectPurple() {
        // FIXED earlier: blue range condition preserved
        return colorSensor.red() >= 150 && colorSensor.red() <= 250 &&
                colorSensor.blue() >= 300 && colorSensor.blue() <= 450 &&
                colorSensor.green() >= 200 && colorSensor.green() <= 300;
    }

    public void leverDown() { leverServo.setPosition(0); }
    public void leverUp() { leverServo.setPosition(0.2); }

    // CHANGED: removed blocking waitMs usage; FSM uses stateTimer now.
    private int degreesToTicks(double deg) {
        return (int)((deg / 360.0) * TICKS_PER_REV);
    }

    // CHANGED: non-blocking start of a single sorter advance "press"
    // This function prepares the next advance step: respects debounce by setting a waiting flag if needed,
    // or sets sorterAdvanceInProgress=true so the main loop will perform PID updates and move.
    private void startNextAdvance() {
        if (sorterPressesRemaining <= 0) return;

        long now = System.currentTimeMillis();
        long sinceLast = now - lastAdvanceTime;
        if (sinceLast < ADVANCE_DEBOUNCE_MS) {
            // need to wait before starting this step — set waiting flags, main loop will begin movement later
            sorterAdvanceWaitingForDebounce = true;
            nextAdvanceStartTime = lastAdvanceTime + ADVANCE_DEBOUNCE_MS;
            return;
        }

        // increment index and set next target (same as original logic)
        currentIndex++;
        if (currentIndex >= positionDegrees.length) {
            currentIndex = 0;
        }

        targetTicks = degreesToTicks(positionDegrees[currentIndex]);
        pid.setSetPoint(targetTicks);

        sorterAdvanceInProgress = true;
        sorterAdvanceWaitingForDebounce = false;
        // do not update lastAdvanceTime here — we update it when movement completes (like original)
    }

    // CHANGED: helper to determine if the sorter (current physical position) is near any forbidden angles.
    // This implements the "deadlimit of between 5 of those" requirement (tolerance in degrees).
    private boolean isSorterNearForbiddenAngles(double toleranceDegrees) {
        // read encoder position and convert to degrees (0-360)
        double ticks = sorterMotor.getCurrentPosition();
        double deg = (ticks / TICKS_PER_REV) * 360.0;
        // normalize into [0,360)
        deg = ((deg % 360.0) + 360.0) % 360.0;

        double[] forbidden = {60.0, 180.0, 300.0};
        for (double f : forbidden) {
            if (Math.abs(angleDifferenceDegrees(deg, f)) <= toleranceDegrees) {
                return true;
            }
        }
        return false;
    }

    // CHANGED: small helper to compute shortest angular difference between two angles in degrees
    private double angleDifferenceDegrees(double a, double b) {
        double diff = Math.abs(a - b) % 360.0;
        if (diff > 180.0) diff = 360.0 - diff;
        return diff;
    }
}
