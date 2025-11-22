package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

@Config
@TeleOp(name = "Turret IMU PID Tuner")
public class TurretImuPidTuner extends OpMode {

    private DcMotor turretMotor;
    private IMU turretImu;

    // PID tunables
    public static double kP = 0.02;
    public static double kI = 0.0;
    public static double kD = 0.0;
    public static double maxPower = 0.4;

    // Target angle (degrees, -180 to 180)
    public static double targetAngleDeg = 0.0;

    // Dpad step size
    public static double angleStep = 5.0;

    private double errorSum = 0.0;
    private double lastError = 0.0;
    private ElapsedTime loopTimer = new ElapsedTime();

    private boolean prevLeft = false;
    private boolean prevRight = false;

    @Override
    public void init() {
        turretMotor = hardwareMap.get(DcMotor.class, "turretMotor");
        turretImu = hardwareMap.get(IMU.class, "turretImu");

        // Adjust this orientation based on how your turret IMU is mounted
        RevHubOrientationOnRobot orientationOnRobot = new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.RIGHT,
                RevHubOrientationOnRobot.UsbFacingDirection.UP
        );
        turretImu.initialize(new IMU.Parameters(orientationOnRobot));

        turretMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        loopTimer.reset();
    }

    @Override
    public void start() {
        // Zero yaw when you start tuning, so your current heading becomes 0°
        turretImu.resetYaw();
    }

    @Override
    public void loop() {
        double dt = loopTimer.seconds();
        loopTimer.reset();

        // Read current yaw (degrees, -180 to 180)
        YawPitchRollAngles ypr = turretImu.getRobotYawPitchRollAngles();
        double currentAngle = ypr.getYaw(AngleUnit.DEGREES);

        // Button controls for stepping target angle
        boolean dpadLeft = gamepad1.dpad_left;
        boolean dpadRight = gamepad1.dpad_right;

        boolean leftJustPressed = dpadLeft && !prevLeft;
        boolean rightJustPressed = dpadRight && !prevRight;

        if (leftJustPressed) {
            targetAngleDeg -= angleStep;
        }
        if (rightJustPressed) {
            targetAngleDeg += angleStep;
        }

        // Normalize target to [-180, 180)
        targetAngleDeg = AngleUnit.normalizeDegrees(targetAngleDeg);

        // Compute smallest signed error using normalizeDegrees
        double error = AngleUnit.normalizeDegrees(targetAngleDeg - currentAngle);

        // PID
        double pTerm = kP * error;

        double dTerm = 0.0;
        if (dt > 0) {
            double errorDelta = error - lastError;
            dTerm = kD * (errorDelta / dt);
        }

        errorSum += error * dt;
        // basic anti-windup
        errorSum = Range.clip(errorSum, -1000, 1000);
        double iTerm = kI * errorSum;

        double rawPower = pTerm + iTerm + dTerm;

        // If close enough, just stop
        if (Math.abs(error) < 1.0) { // within 1 degree
            rawPower = 0.0;
            errorSum = 0.0;
        }

        double power = Range.clip(rawPower, -maxPower, maxPower);
        turretMotor.setPower(power);

        telemetry.addLine("=== Turret IMU PID Tuner ===");
        telemetry.addData("Current Angle (deg)", "%.1f", currentAngle);
        telemetry.addData("Target  Angle (deg)", "%.1f", targetAngleDeg);
        telemetry.addData("Error (deg)", "%.2f", error);
        telemetry.addData("Motor Power", "%.3f", power);
        telemetry.addData("kP", kP);
        telemetry.addData("kI", kI);
        telemetry.addData("kD", kD);
        telemetry.addData("MaxPower", maxPower);
        telemetry.addData("AngleStep", angleStep);
        telemetry.addLine("Controls: Dpad Left/Right = change target angle, Dashboard = tune gains");
        telemetry.update();

        prevLeft = dpadLeft;
        prevRight = dpadRight;
        lastError = error;
    }

    @Override
    public void stop() {
        turretMotor.setPower(0);
    }
}
