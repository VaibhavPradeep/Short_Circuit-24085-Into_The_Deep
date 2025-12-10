// Shooter9kVelocityTuner with RT>0.75 gating added only where needed
package org.firstinspires.ftc.teamcode;

import static com.qualcomm.robotcore.hardware.DcMotor.RunMode.RUN_USING_ENCODER;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.hardware.dfrobot.HuskyLens;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcore.internal.system.Deadline;
import java.util.concurrent.TimeUnit;

@Config
@TeleOp(name = "Shooter9kVelocityTuner")
public class Shooter9kVelocityTester extends OpMode {

    private DcMotorEx shootingMotor;
    private Servo pitchServo;

    public static boolean useVelocityControl = true;
    public static double shooterPower = 0.0;

    public static double targetWheelRpm = 5700.0;
    public static int motorTicksPerRev = 28;
    public static double gearRatioWheelOverMotor = 1.5;

    public static double kPs = 0.01;
    public static double kIs = 0.0;
    public static double kDs = 0.0;
    public static double kFs = 0.0;

    public static double rpmTolerance = 100.0;

    public static double servoPosition = 0.0;
    public static double leverPos = 0.0;

    private PIDController shooterPid;

    Servo leverServo;

    @Override
    public void init() {
        shootingMotor = hardwareMap.get(DcMotorEx.class, "shootingMotor");
        pitchServo = hardwareMap.get(Servo.class, "pitchServo");
        leverServo = hardwareMap.get(Servo.class, "leverServo");

        shootingMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        shootingMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        shootingMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        shooterPid = new PIDController(kPs, kIs, kDs);

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
    }

    @Override
    public void loop() {
        pitchServo.setPosition(servoPosition);
        leverServo.setPosition(leverPos);

        // === ADDED: Right Trigger Safety Gate ===
        boolean shooterEnabled = gamepad1.right_trigger > 0.75;

        if (!shooterEnabled) {
            shootingMotor.setPower(0);
            telemetry.addLine("Shooter OFF (Hold RT > 0.75)");
            telemetry.update();
            return;
        }
        // =======================================

        if (!useVelocityControl) {
            shootingMotor.setPower(shooterPower);
            telemetry.addLine("=== Shooter Power Mode ===");
            telemetry.addData("Power", shooterPower);
            telemetry.update();
            return;
        }

        double targetMotorRpm = targetWheelRpm / gearRatioWheelOverMotor;
        double actualMotorTicksPerSec = shootingMotor.getVelocity();
        double actualMotorRpm = (actualMotorTicksPerSec * 60.0) / motorTicksPerRev;
        double actualWheelRpm = actualMotorRpm * gearRatioWheelOverMotor;
        double errorWheelRpm = targetWheelRpm - actualWheelRpm;

        shooterPid.setPID(kPs, kIs, kDs);

        double pidOutput = shooterPid.calculate(actualWheelRpm, targetWheelRpm);
        double ff = kFs * targetWheelRpm;
        double power = pidOutput + ff;
        power = Math.max(0.0, Math.min(1.0, power));

        shootingMotor.setPower(power);

        boolean atSpeed = Math.abs(errorWheelRpm) <= rpmTolerance;

        telemetry.addLine("=== Shooter Velocity PID Mode ===");
        telemetry.addData("Target wheel RPM", targetWheelRpm);
        telemetry.addData("Actual wheel RPM", actualWheelRpm);
        telemetry.addData("Error", errorWheelRpm);
        telemetry.addData("At Speed?", atSpeed);
        telemetry.addData("Power", power);
        telemetry.update();
    }

    @Override
    public void stop() {
        shootingMotor.setPower(0);
    }
}
