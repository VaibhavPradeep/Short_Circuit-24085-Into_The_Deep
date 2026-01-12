package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.IMU;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

@Config
@TeleOp(name = "Turret PID IMU + Encoder Switchback")
public class TurretPIDEncoderSwitchback extends OpMode {

    // ---------------- PID ----------------
    private PIDController controller;

    public static double p = 0.019;
    public static double i = 0.0;
    public static double d = 0.001;
    public static double kFF = 0.1;

    public static double targetAngle = 90; // degrees

    // ---------------- ENCODER LIMITS ----------------
    // 435 RPM GoBILDA (28 CPR) with 20T -> 150T gear ≈ 7.5:1
    public static int TICKS_PER_REV = 210;
    public static int MAX_TICKS = 210;        // ±360°
    public static int LIMIT_MARGIN = 25;       // switchback buffer

    // ---------------- HARDWARE ----------------
    private DcMotor rotationMotor;
    private IMU turretImu;

    // ---------------- IMU UNWRAP ----------------
    private double lastYawDeg = 0.0;
    private double unwrappedYawDeg = 0.0;
    private double yawOffsetDeg = 0.0;

    @Override
    public void init() {
        controller = new PIDController(p, i, d);
        telemetry = new MultipleTelemetry(
                telemetry, FtcDashboard.getInstance().getTelemetry()
        );

        rotationMotor = hardwareMap.get(DcMotor.class, "rotationMotor");
        turretImu = hardwareMap.get(IMU.class, "turretImu");

        // Encoder setup
        rotationMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rotationMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rotationMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // IMU orientation
        RevHubOrientationOnRobot orientation = new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.RIGHT,
                RevHubOrientationOnRobot.UsbFacingDirection.UP
        );
        turretImu.initialize(new IMU.Parameters(orientation));
        turretImu.resetYaw();

        // Initialize IMU unwrap
        YawPitchRollAngles angles = turretImu.getRobotYawPitchRollAngles();
        lastYawDeg = angles.getYaw(AngleUnit.DEGREES);
        unwrappedYawDeg = lastYawDeg;
        yawOffsetDeg = lastYawDeg;
    }

    @Override
    public void loop() {
        controller.setPID(p, i, d);

        // ---------- IMU UNWRAP ----------
        YawPitchRollAngles orientation = turretImu.getRobotYawPitchRollAngles();
        double yaw = orientation.getYaw(AngleUnit.DEGREES);

        double delta = yaw - lastYawDeg;
        if (delta > 180) delta -= 360;
        if (delta < -180) delta += 360;

        unwrappedYawDeg += delta;
        lastYawDeg = yaw;

        double turretAngle = unwrappedYawDeg - yawOffsetDeg;

        // ---------- ENCODER POSITION ----------
        int encoderPos = rotationMotor.getCurrentPosition();

        // ---------- SWITCHBACK TARGET ----------
        double adjustedTarget = targetAngle;

        // Near +360° → force reverse
        if (encoderPos > MAX_TICKS - LIMIT_MARGIN) {
            adjustedTarget = targetAngle - 360;
        }

        // Near -360° → force forward
        else if (encoderPos < -MAX_TICKS + LIMIT_MARGIN) {
            adjustedTarget = targetAngle + 360;
        }

        // ---------- PID CONTROL ----------
        double error = adjustedTarget - turretAngle;
        double pid = controller.calculate(turretAngle, adjustedTarget);
        double ff = Math.copySign(kFF, error);

        double power = pid + ff;

        // ---------- HARD SAFETY CUTOFF ----------
        if ((encoderPos >= MAX_TICKS && power > 0) ||
                (encoderPos <= -MAX_TICKS && power < 0)) {
            power = 0;
        }

        rotationMotor.setPower(
                Math.max(-1.0, Math.min(1.0, power))
        );

        // ---------- TELEMETRY ----------
        telemetry.addData("Turret Angle (deg)", turretAngle);
        telemetry.addData("Encoder", encoderPos);
        telemetry.addData("Target", targetAngle);
        telemetry.addData("Adjusted Target", adjustedTarget);
        telemetry.addData("Motor Power", power);
        telemetry.update();
    }
}
