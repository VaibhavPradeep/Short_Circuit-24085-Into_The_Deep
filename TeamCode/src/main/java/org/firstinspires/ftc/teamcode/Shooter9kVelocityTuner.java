package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

@Config
@TeleOp(name = "Shooter9kVelocityTuner")
public class Shooter9kVelocityTuner extends OpMode {

    // Shooter motor (with encoder)
    private DcMotorEx shooterMotor;

    // Shooter servo
    private Servo shooterServo;

    // --- THROUGH-BORE ENCODER ---
    private DcMotorEx thruEncoder;  // MUST be mapped in robot config
    public static boolean testEncoder = false;
    private int lastEncoderPos = 0;

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

    // --- Servo Control ---
    public static double servoPosition = 0.0; // 0.0 → 1.0

    // For shooter velocity measurement
    private ElapsedTime velTimer = new ElapsedTime();
    private int lastPosition = 0;

    // --- FTC Dashboard ---
    private FtcDashboard dashboard;

    @Override
    public void init() {
        shooterMotor = hardwareMap.get(DcMotorEx.class, "shooterMotor");
        shooterServo = hardwareMap.get(Servo.class, "shooterServo");

        // Thru bore encoder mapped as a motor device
        // Ex: map "thruEncoder" to a port with ONLY the encoder plugged in.
        thruEncoder = hardwareMap.get(DcMotorEx.class, "thruEncoder");
        thruEncoder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        thruEncoder.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        shooterMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        shooterMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        shooterMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        shooterServo.setPosition(servoPosition);

        velTimer.reset();
        lastPosition = shooterMotor.getCurrentPosition();
        lastEncoderPos = thruEncoder.getCurrentPosition();

        // --- Setup FTC Dashboard ---
        dashboard = FtcDashboard.getInstance();
        telemetry = new MultipleTelemetry(telemetry, dashboard.getTelemetry());
    }

    @Override
    public void loop() {

        // --- ENCODER TEST MODE ---
        if (testEncoder) {
            int pos = thruEncoder.getCurrentPosition();
            int delta = pos - lastEncoderPos;
            lastEncoderPos = pos;

            double ticksPerRev = 8192.0; // REV through-bore default
            double revolutions = pos / ticksPerRev;
            double degrees = (pos % ticksPerRev) * (360.0 / ticksPerRev);

            telemetry.addLine("=== THROUGH-BORE ENCODER TEST ===");
            telemetry.addData("Raw Ticks", pos);
            telemetry.addData("Delta Ticks", delta);
            telemetry.addData("Revolutions", "%.3f", revolutions);
            telemetry.addData("Degrees", "%.1f", degrees);
            telemetry.addData("Spin the encoder by hand", "");

            // Also send to Dashboard as a packet (optional)
            TelemetryPacket packet = new TelemetryPacket();
            packet.put("rawTicks", pos);
            packet.put("deltaTicks", delta);
            packet.put("revolutions", revolutions);
            packet.put("degrees", degrees);
            dashboard.sendTelemetryPacket(packet);

            telemetry.update();
            return;
        }

        // Update servo
        shooterServo.setPosition(servoPosition);

        // ============= POWER MODE =============
        if (!useVelocityControl) {

            shooterMotor.setPower(shooterPower);

            telemetry.addLine("=== Shooter Power Mode ===");
            telemetry.addData("Power", "%.2f", shooterPower);
            telemetry.addData("Servo Position", "%.2f", servoPosition);

            if (shooterPower >= 1.0) {
                telemetry.addLine("⚠️ MOTOR AT MAX POWER ⚠️");
            }

            // Optional: compute velocity here too so you can still graph it in power mode
            double dt = velTimer.seconds();
            velTimer.reset();

            int currentPos = shooterMotor.getCurrentPosition();
            int deltaTicks = currentPos - lastPosition;
            lastPosition = currentPos;

            double actualMotorTicksPerSec = (dt > 0) ? (deltaTicks / dt) : 0.0;
            double actualMotorRpm = (actualMotorTicksPerSec * 60.0) / motorTicksPerRev;
            double actualWheelRpm = actualMotorRpm * gearRatioWheelOverMotor;

            telemetry.addData("Actual wheel RPM", "%.1f", actualWheelRpm);

            // --- Dashboard Graph Packet (Power Mode) ---
            TelemetryPacket packet = new TelemetryPacket();
            packet.put("actualWheelRpm", actualWheelRpm);
            packet.put("shooterPower", shooterPower);
            dashboard.sendTelemetryPacket(packet);

            telemetry.update();
            return;
        }

        // ============= VELOCITY MODE =============

        double targetMotorRpm = targetWheelRpm / gearRatioWheelOverMotor;
        double targetMotorTicksPerSec = (targetMotorRpm * motorTicksPerRev) / 60.0;

        shooterMotor.setVelocityPIDFCoefficients(kP, kI, kD, kF);
        shooterMotor.setVelocity(targetMotorTicksPerSec);

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
        telemetry.addData("Servo Position", "%.2f", servoPosition);

        boolean motorMaxedOut = false;

        if (actualMotorTicksPerSec + 20 < targetMotorTicksPerSec &&
                shooterMotor.getPower() >= 0.99) {
            motorMaxedOut = true;
        }

        if (motorMaxedOut) {
            telemetry.addLine("⚠️ MOTOR AT MAX OUTPUT — CANNOT GO FASTER ⚠️");
        }

        // --- Dashboard Graph Packet (Velocity Mode) ---
        TelemetryPacket packet = new TelemetryPacket();
        packet.put("targetWheelRpm", targetWheelRpm);
        packet.put("actualWheelRpm", actualWheelRpm);
        packet.put("errorWheelRpm", errorWheelRpm);
        dashboard.sendTelemetryPacket(packet);

        telemetry.update();
    }

    @Override
    public void stop() {
        shooterMotor.setPower(0);
    }
}
