package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp(name="Spindexer_FTCLib_PID_Iterative_NoSleep", group="Main")
public class Spindexer_FTCLib_PID_Iterative_NoSleep extends OpMode {

    private DcMotor spindexer;
    private PIDController pid;

    private static final double TICKS_PER_REV = 537.6;  // 312 RPM motor

    // Positions around spindexer circle (in degrees)
    // CHANGE THESE BASED ON YOUR REAL SPINDEXER GEOMETRY
    private double[] positionDegrees = { 0, 60, 120, 180, 240, 300 };

    private int currentIndex = 0;
    private int targetTicks = 0;

    // PID gains — tune as needed
    private double kP = 0.015;
    private double kI = 0.0;
    private double kD = 0.0003;

    // For button debounce
    private long lastAdvanceTime = 0;
    private static final long ADVANCE_DEBOUNCE_MS = 200;

    @Override
    public void init() {
        spindexer = hardwareMap.get(DcMotor.class, "spindexer");
        spindexer.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        spindexer.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        spindexer.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        pid = new PIDController(kP, kI, kD);

        // initial target
        targetTicks = degreesToTicks(positionDegrees[currentIndex]);
        pid.setSetPoint(targetTicks);

        telemetry.addData("status", "initialized");
    }

    @Override
    public void loop() {
        long now = System.currentTimeMillis();

        // One-button advance (with debounce)
        if (gamepad1.right_bumper && (now - lastAdvanceTime) > ADVANCE_DEBOUNCE_MS) {
            currentIndex++;
            if (currentIndex >= positionDegrees.length) {
                currentIndex = 0;
            }
            targetTicks = degreesToTicks(positionDegrees[currentIndex]);
            pid.setSetPoint(targetTicks);

            lastAdvanceTime = now;
        }

        // Read current position
        double currentPos = spindexer.getCurrentPosition();

        // PID controller: compute power
        double power = pid.calculate(currentPos);

        // Deadband / tolerance (to avoid jitter near target)
        if (Math.abs(targetTicks - currentPos) < 10) {
            power = 0;
        }

        // Clamp to safe motor power
        power = Math.max(-0.6, Math.min(0.6, power));

        spindexer.setPower(power);

        // Telemetry for tuning/debugging
        telemetry.addData("Cur ticks", currentPos);
        telemetry.addData("Tgt ticks", targetTicks);
        telemetry.addData("Index", currentIndex);
        telemetry.addData("Error", targetTicks - currentPos);
        telemetry.addData("Power", power);
        telemetry.update();
    }

    @Override
    public void stop() {
        spindexer.setPower(0);
    }

    private int degreesToTicks(double deg) {
        return (int)((deg / 360.0) * TICKS_PER_REV);
    }
}
