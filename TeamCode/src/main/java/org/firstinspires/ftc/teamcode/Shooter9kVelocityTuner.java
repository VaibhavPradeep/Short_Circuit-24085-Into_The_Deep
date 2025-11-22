package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

@Config
@TeleOp(name = "Shooter9kVelocityTuner")
public class Shooter9kVelocityTuner extends OpMode {

    // Shooter motor (with encoder)
    private DcMotorEx shooterMotor;

    // === Dashboard Tunables ===

    // --- MODE SELECT ---
    public static boolean useVelocityControl = true;   // true = velocity mode, false = power mode

    // --- Power Mode ---
    public static double shooterPower = 0.0;           // 0 to 1.0

    // --- Velocity Mode ---
    public static double targetWheelRpm = 9000.0;
    public static int motorTicksPerRev = 28;
    public static double gearRatioWheelOverMotor = 1.5;

    public static double kP = 0.0;
    public static double kI = 0.0;
    public static double kD = 0.0;
    public static double kF = 0.0;

    // For measuring velocity
    private ElapsedTime velTimer = new ElapsedTime();
    private int lastPosition = 0;

    @Override
    public void init() {
        shooterMotor = hardwareMap.get(DcMotorEx.class, "shooterMotor");

        shooterMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        shooterMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        shooterMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        velTimer.reset();
        lastPosition = shooterMotor.getCurrentPosition();
    }

    @Override
    public void loop() {

        if (!useVelocityControl) {
            // ============= POWER MODE =============
            shooterMotor.setPower(shooterPower);

            telemetry.addLine("=== Shooter Power Mode ===");
            telemetry.addData("Power", "%.2f", shooterPower);
            telemetry.addLine("Switch useVelocityControl to true for velocity control.");
            telemetry.update();
            return;
        }

        // ============= VELOCITY MODE =============

        // Convert wheel RPM → motor RPM → ticks/sec
        double targetMotorRpm = targetWheelRpm / gearRatioWheelOverMotor;
        double targetMotorTicksPerSec = (targetMotorRpm * motorTicksPerRev) / 60.0;

        // Apply PIDF
        shooterMotor.setVelocityPIDFCoefficients(kP, kI, kD, kF);

        // Set motor velocity
        shooterMotor.setVelocity(targetMotorTicksPerSec);

        // Measure actual velocity
        double dt = velTimer.seconds();
        velTimer.reset();

        int currentPos = shooterMotor.getCurrentPosition();
        int deltaTicks = currentPos - lastPosition;
        lastPosition = currentPos;

        double actualMotorTicksPerSec = (dt > 0) ? (deltaTicks / dt) : 0.0;
        double actualMotorRpm = (actualMotorTicksPerSec * 60.0) / motorTicksPerRev;
        double actualWheelRpm = actualMotorRpm * gearRatioWheelOverMotor;
        double errorWheelRpm = targetWheelRpm - actualWheelRpm;

        telemetry.addLine("=== Shooter Velocity Mode ===");
        telemetry.addData("Target wheel RPM", "%.1f", targetWheelRpm);
        telemetry.addData("Actual wheel RPM", "%.1f", actualWheelRpm);
        telemetry.addData("Error wheel RPM", "%.1f", errorWheelRpm);
        telemetry.addLine();
        telemetry.addData("kP", kP);
        telemetry.addData("kI", kI);
        telemetry.addData("kD", kD);
        telemetry.addData("kF", kF);
        telemetry.addLine();
        telemetry.addData("Mode", "Velocity Control Active");
        telemetry.update();
    }

    @Override
    public void stop() {
        shooterMotor.setPower(0);
    }
}
