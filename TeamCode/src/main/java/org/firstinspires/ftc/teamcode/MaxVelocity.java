package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

@Config
@TeleOp(name = "max velocity")
public class MaxVelocity extends OpMode {
    private DcMotorEx shootingMotor;
    private Servo pitchServo;

    public static double kPs = 0.0;
    public static double kIs = 0.0;
    public static double kDs = 0.0;

    private PIDController controller;

    public static double target = 0;

    Servo leverServo;

    @Override
    public void init() {
        shootingMotor = hardwareMap.get(DcMotorEx.class, "shootingMotor");
        pitchServo = hardwareMap.get(Servo.class, "pitchServo");
        leverServo = hardwareMap.get(Servo.class, "leverServo");

        controller = new PIDController(kPs,kIs,kDs);
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());


        pitchServo.setDirection(Servo.Direction.REVERSE);
        pitchServo.setPosition(0);

        shootingMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        shootingMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        shootingMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
    }

    @Override
    public void start() {
        pitchServo.setPosition(0.42);
    }

    @Override
    public void loop() {
        shootingMotor.setPower(1);

        telemetry.addData("Pos: ", shootingMotor.getVelocity());
        telemetry.addData("target: ", target);
        telemetry.update();
    }

}