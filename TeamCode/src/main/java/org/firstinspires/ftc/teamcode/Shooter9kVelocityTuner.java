package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

@Config
@TeleOp(name = "Shooter 9k Velocity Tuner")
public class Shooter9kVelocityTuner extends OpMode {

    // Shooter motor (with encoder)
    private DcMotorEx shooterMotor;

    // === Tunables in Dashboard ===

    // Desired *wheel* RPM (after 1.5:1 gear-up)
    public static double targetWheelRpm = 9000.0;

    // Motor encoder ticks per motor shaft revolution
    // (Typical goBILDA embedded encoder is 28)
    public static int motorTicksPerRev = 28;

    // External gear ratio: wheelRPM = motorRPM * gearRatioWheelOverMotor
    // Example: 1.5 means wheel spins 1.5x faster than motor (6k -> 9k)
    public static double gearRatioWheelOverMotor = 1.5;

    // PIDF coefficients for the motor controller
    public static double kP = 0.0;
    public static double kI = 0.0;
    public static double kD = 0.0;
    public static double kF = 0.0;

    // For measuring velocity ourselves
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
        // 1) Convert target *wheel* RPM -> target *motor* ticks/sec

        // wheelRPM = motorRPM * gearRatio
        // => motorRPM = wheelRPM / gearRatio
        double targetMotorRpm = targetWheelRpm / gearRatioWheelOverMotor;

        // motorTicksPerSec = motorRPM * motorTicksPerRev / 60
        double targetMotorTicksPerSec = (targetMotorRpm * motorTicksPerRev) / 60.0;

        // 2) Apply PIDF coefficients (Dashboard can tweak these live)
        shooterMotor.setVelocityPIDFCoefficients(kP, kI, kD, kF);

        // 3) Command the motor to that velocity (in ticks per second)
        shooterMotor.setVelocity(targetMotorTicksPerSec);

        // 4) Measure actual motor velocity
        double dt = velTimer.seconds();
        velTimer.reset();

        int currentPos = shooterMotor.getCurrentPosition();
        int deltaTicks = currentPos - lastPosition;
        lastPosition = currentPos;

        double actualMotorTicksPerSec = (dt > 0) ? (deltaTicks / dt) : 0.0;

        // Convert back to motor RPM and wheel RPM for telemetry
        double actualMotorRpm = (actualMotorTicksPerSec * 60.0) / motorTicksPerRev;
        double actualWheelRpm = actualMotorRpm * gearRatioWheelOverMotor;

        double errorWheelRpm = targetWheelRpm - actualWheelRpm;

        telemetry.addLine("=== Shooter 9k Velocity Tuner ===");
        telemetry.addData("Target wheel RPM", "%.1f", targetWheelRpm);
        telemetry.addData("Actual wheel RPM", "%.1f", actualWheelRpm);
        telemetry.addData("Error wheel RPM", "%.1f", errorWheelRpm);
        telemetry.addLine();
        telemetry.addData("Target motor RPM", "%.1f", targetMotorRpm);
        telemetry.addData("Actual motor RPM", "%.1f", actualMotorRpm);
        telemetry.addData("Target motor ticks/sec", "%.1f", targetMotorTicksPerSec);
        telemetry.addData("Actual motor ticks/sec", "%.1f", actualMotorTicksPerSec);
        telemetry.addLine();
        telemetry.addData("kP", kP);
        telemetry.addData("kI", kI);
        telemetry.addData("kD", kD);
        telemetry.addData("kF", kF);
        telemetry.update();
    }

    @Override
    public void stop() {
        shooterMotor.setPower(0);
    }
}
