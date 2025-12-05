package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;

@Config
@TeleOp(name="PIDF_Tuner_AllMotors", group="Tuning")
public class PIDFTunerAllMotors extends LinearOpMode {

    public static double kP = 1.0, kI = 0.0, kD = 0.0, kF = 0.0;
    public static double power = 0.5;

    @Override
    public void runOpMode() throws InterruptedException {

        DcMotorEx intake = hardwareMap.get(DcMotorEx.class, "intake");
        DcMotorEx shooter = hardwareMap.get(DcMotorEx.class, "shooter");
        DcMotorEx spindex = hardwareMap.get(DcMotorEx.class, "spindexer");

        intake.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        shooter.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        spindex.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);

        DcMotorEx current = intake;

        FtcDashboard dash = FtcDashboard.getInstance();

        waitForStart();

        while (opModeIsActive()) {

            if (gamepad1.x) current = intake;
            if (gamepad1.y) current = shooter;
            if (gamepad1.b) current = spindex;

            current.setVelocityPIDFCoefficients(kP, kI, kD, kF);

            if (gamepad1.a) current.setPower(power);
            else current.setPower(0);

            TelemetryPacket pkt = new TelemetryPacket();
            pkt.put("motor", current.getDeviceName());
            pkt.put("velocity", current.getVelocity());
            dash.sendTelemetryPacket(pkt);

            telemetry.addData("Motor", current.getDeviceName());
            telemetry.addData("Velocity", current.getVelocity());
            telemetry.update();
        }
    }
}
