package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@Config
@TeleOp(name = " PIDForDR4B")
public class PIDForDR4B extends LinearOpMode {
    private PIDController controller;
    public static double p = 0, i = 0, d = 0;
    public static double f = 0;

    public static int target = 0;
    private final double ticks_in_degree = 537.6 / 360;
    private DcMotor motor1;
    private DcMotor motor2;

    @Override
    public void runOpMode() throws InterruptedException {
        controller = new PIDController(p, i ,d);
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        // Initialize motors from the hardware map
        motor1 = hardwareMap.get(DcMotor.class, "motor1");
        motor2 = hardwareMap.get(DcMotor.class, "motor2");

        motor2.setDirection(DcMotorSimple.Direction.REVERSE);

        waitForStart();

        while (opModeIsActive()) {
            controller.setPID(p, i, d);

            int motor1CurrentPosition = motor1.getCurrentPosition();
            int motor2CurrentPosition = motor2.getCurrentPosition();
            double pid1 = controller.calculate(motor1CurrentPosition, target);
            double pid2 = controller.calculate(motor2CurrentPosition, target);
            double ff = Math.cos(Math.toRadians(target / ticks_in_degree)) * f;

            double power1 = pid1 * ff;
            double power2 = pid2 * ff;

            motor1.setPower(power1);
            motor2.setPower(power2);

            telemetry.addData("motor1 pos", motor1CurrentPosition);
            telemetry.addData("motor2 pos", motor2CurrentPosition);
            telemetry.addData("target", target);
            telemetry.update();
        }

    }
}
