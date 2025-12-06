package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

@Config
@TeleOp(name = "Havish's Spindexer Position Tuner")
public class hu_SpindexerPositionTuner extends OpMode {

    // ---- HARDWARE ----
    private CRServo spindexerServo;   // Axon Servo Max MK2 in continuous mode
    private DcMotor spindexerEncoder; // REV Through-Bore encoder plugged into motor port

    // ---- PID & GENERAL SETTINGS (TUNABLE IN DASHBOARD) ----
    public static double kP = 0.003;      // start small; tune up/down in Dashboard
    public static double kI = 0.0;        // likely 0 for this use
    public static double kD = 0.0;        // optional; start at 0
    public static double maxPower = 0.4;  // clamp CR servo power; lower if overshoot
    public static int ticksPerRev = 8192; // REV Through-Bore CPR
    public static double gearRatio = 1.0; // encoder:axle ratio (1:1 since encoder is on axle)
    public static int positionTolerance = 30; // ticks error allowed before we consider "at position"

    // ---- PRESET POSITIONS (TUNABLE IN DASHBOARD) ----
    // Gap 1
    public static int GAP1_INTAKE_TICKS  = 0;
    public static int GAP1_OUTTAKE_TICKS = 1000;

    // Gap 2
    public static int GAP2_INTAKE_TICKS  = 2700;
    public static int GAP2_OUTTAKE_TICKS = 3700;

    // Gap 3
    public static int GAP3_INTAKE_TICKS  = 5400;
    public static int GAP3_OUTTAKE_TICKS = 6400;

    // Which preset we are currently driving to (0..5)
    // 0 = G1 intake, 1 = G1 outtake, 2 = G2 intake, 3 = G2 outtake, 4 = G3 intake, 5 = G3 outtake
    public static int activePresetIndex = 0;

    // ---- STEP SIZE FOR MANUAL NUDGING OF TARGET (OPTIONAL) ----
    public static int manualStepTicks = 50; // change target by this many ticks using dpad up/down

    // ---- INTERNAL STATE ----
    private int targetTicks = 0;
    private int lastError = 0;
    private double errorSum = 0.0;
    private ElapsedTime loopTimer = new ElapsedTime();

    private boolean prevDpadLeft = false;
    private boolean prevDpadRight = false;
    private boolean prevDpadUp = false;
    private boolean prevDpadDown = false;
    private boolean prevA = false;

    @Override
    public void init() {
        spindexerServo   = hardwareMap.get(CRServo.class, "spindexerServo");
        spindexerEncoder = hardwareMap.get(DcMotor.class, "spindexerEncoder");

        // Reset encoder
        spindexerEncoder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        spindexerEncoder.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // If direction is backwards, you can flip:
        // spindexerServo.setDirection(CRServo.Direction.REVERSE);

        loopTimer.reset();
    }

    @Override
    public void loop() {
        double dt = loopTimer.seconds();
        loopTimer.reset();

        int currentTicks = spindexerEncoder.getCurrentPosition();

        // ------------- HANDLE INPUTS / PRESET SELECTION -------------

        boolean dpadLeft  = gamepad1.dpad_left;
        boolean dpadRight = gamepad1.dpad_right;
        boolean dpadUp    = gamepad1.dpad_up;
        boolean dpadDown  = gamepad1.dpad_down;
        boolean a         = gamepad1.a;

        boolean leftJustPressed  = dpadLeft && !prevDpadLeft;
        boolean rightJustPressed = dpadRight && !prevDpadRight;
        boolean upJustPressed    = dpadUp && !prevDpadUp;
        boolean downJustPressed  = dpadDown && !prevDpadDown;
        boolean aJustPressed     = a && !prevA;

        // Cycle through presets with dpad left/right
        if (leftJustPressed) {
            activePresetIndex--;
        }
        if (rightJustPressed) {
            activePresetIndex++;
        }
        activePresetIndex = Range.clip(activePresetIndex, 0, 5);

        // Optional: fine-adjust current target with dpad up/down
        if (upJustPressed) {
            targetTicks += manualStepTicks;
        }
        if (downJustPressed) {
            targetTicks -= manualStepTicks;
        }

        // A: set whatever encoder is now as the base for the current preset
        if (aJustPressed) {
            switch (activePresetIndex) {
                case 0:
                    GAP1_INTAKE_TICKS = currentTicks;
                    break;
                case 1:
                    GAP1_OUTTAKE_TICKS = currentTicks;
                    break;
                case 2:
                    GAP2_INTAKE_TICKS = currentTicks;
                    break;
                case 3:
                    GAP2_OUTTAKE_TICKS = currentTicks;
                    break;
                case 4:
                    GAP3_INTAKE_TICKS = currentTicks;
                    break;
                case 5:
                    GAP3_OUTTAKE_TICKS = currentTicks;
                    break;
            }
        }

        // X: zero encoder and shift all presets relative to that (optional).
        // If you want a consistent zero spot you go to and then re-measure, you can add logic here.
        if (gamepad1.x) {
            spindexerEncoder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            spindexerEncoder.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            errorSum = 0;
            lastError = 0;
        }

        // ------------- PICK TARGET TICKS FROM ACTIVE PRESET -------------

        switch (activePresetIndex) {
            case 0:
                targetTicks = GAP1_INTAKE_TICKS;
                break;
            case 1:
                targetTicks = GAP1_OUTTAKE_TICKS;
                break;
            case 2:
                targetTicks = GAP2_INTAKE_TICKS;
                break;
            case 3:
                targetTicks = GAP2_OUTTAKE_TICKS;
                break;
            case 4:
                targetTicks = GAP3_INTAKE_TICKS;
                break;
            case 5:
                targetTicks = GAP3_OUTTAKE_TICKS;
                break;
        }

        // ------------- PID CONTROL (MAIN PART) -------------

        int error = targetTicks - currentTicks;

        // Proportional term
        double pTerm = kP * error;

        // Derivative term
        double dTerm = 0.0;
        if (dt > 0) {
            int errorDelta = error - lastError;
            dTerm = kD * (errorDelta / dt);
        }
        lastError = error;

        // Integral term (basic anti-windup)
        errorSum += error * dt;
        errorSum = Range.clip(errorSum, -10000, 10000);
        double iTerm = kI * errorSum;

        double rawPower = pTerm + dTerm + iTerm;

        // If close enough, stop and zero integral
        if (Math.abs(error) < positionTolerance) {
            rawPower = 0.0;
            errorSum = 0.0;
        }

        // Clamp power for safety / smoothness
        double power = Range.clip(rawPower, -maxPower, maxPower);

        spindexerServo.setPower(power);

        // ------------- TELEMETRY -------------

        double rotations = currentTicks / (ticksPerRev * gearRatio);
        double degrees   = rotations * 360.0;

        double targetRotations = targetTicks / (ticksPerRev * gearRatio);
        double targetDegrees   = targetRotations * 360.0;

        telemetry.addLine("=== Spindexer Position Tuner ===");
        telemetry.addData("Active preset index", activePresetIndex);
        telemetry.addData("Preset name",
                presetNameFromIndex(activePresetIndex));
        telemetry.addData("Current ticks", currentTicks);
        telemetry.addData("Target  ticks", targetTicks);
        telemetry.addData("Error (ticks)", error);
        telemetry.addData("Current deg", "%.1f", degrees);
        telemetry.addData("Target  deg", "%.1f", targetDegrees);
        telemetry.addData("Servo power", "%.3f", power);
        telemetry.addLine();
        telemetry.addData("kP/kI/kD", "%.4f / %.4f / %.4f", kP, kI, kD);
        telemetry.addData("MaxPower", "%.2f", maxPower);
        telemetry.addData("Tolerance (ticks)", positionTolerance);
        telemetry.addData("ManualStepTicks", manualStepTicks);
        telemetry.addLine();
        telemetry.addLine("Controls:");
        telemetry.addLine("  Dpad Left/Right = cycle presets (0..5)");
        telemetry.addLine("  Dpad Up/Down    = nudge target +/- stepTicks");
        telemetry.addLine("  A               = save CURRENT ticks into active preset");
        telemetry.addLine("  X               = zero encoder");
        telemetry.update();

        // Save button states
        prevDpadLeft  = dpadLeft;
        prevDpadRight = dpadRight;
        prevDpadUp    = dpadUp;
        prevDpadDown  = dpadDown;
        prevA         = a;
    }

    private String presetNameFromIndex(int i) {
        switch (i) {
            case 0: return "GAP1_INTAKE";
            case 1: return "GAP1_OUTTAKE";
            case 2: return "GAP2_INTAKE";
            case 3: return "GAP2_OUTTAKE";
            case 4: return "GAP3_INTAKE";
            case 5: return "GAP3_OUTTAKE";
            default: return "UNKNOWN";
        }
    }
}
