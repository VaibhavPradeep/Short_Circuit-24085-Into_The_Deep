package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "PIDFShootingTuner")
public class PIDFShootingTuner extends OpMode {

    public DcMotorEx shootingMotor;

    Servo leverServo;
    public double highVelocity = 1500;
    public double lowVelocity = 900;
    public double curTargetVelocity = highVelocity;
    public double f = 0;
    public double p = 0;
    public double[] stepSizes = {10.0, 1.0, 0.1, 0.01, 0.001};
    public int stepIndex = 1;

    @Override
    public void init() {
        shootingMotor = hardwareMap.get(DcMotorEx.class, "shootingMotor");
        leverServo = hardwareMap.get(Servo.class, "leverServo");
        shootingMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        shootingMotor.setDirection(DcMotor.Direction.FORWARD);

        PIDFCoefficients pidf = new PIDFCoefficients(p, 0, 0, f);
        shootingMotor.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidf);

        telemetry.addLine("Init complete");
    }

    @Override
    public void loop() {
        // Change target velocity with Y button
        if (gamepad1.y) {
            if (curTargetVelocity == highVelocity) {
                curTargetVelocity = lowVelocity;
            } else {
                curTargetVelocity = highVelocity;
            }
        }

        if (gamepad1.b) {
            stepIndex = (stepIndex + 1) % stepSizes.length;
        }

        // Adjust F with dpad left or right
        if (gamepad1.dpad_left) {
            f -= stepSizes[stepIndex];
        }
        if (gamepad1.dpad_right) {
            f += stepSizes[stepIndex];
        }

        // Adjust P with dpad up or down
        if (gamepad1.dpad_up) {
            p += stepSizes[stepIndex];
        }
        if (gamepad1.dpad_down) {
            p -= stepSizes[stepIndex];
        }

        if (gamepad2.dpad_up) {
            leverServo.setPosition(0.2);
        } else {
            leverServo.setPosition(0);
        }

        PIDFCoefficients newPidf = new PIDFCoefficients(p, 0, 0, f);
        shootingMotor.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, newPidf);

        shootingMotor.setVelocity(curTargetVelocity);

        double curVelocity = shootingMotor.getVelocity();
        double error = curTargetVelocity - curVelocity;


        telemetry.addData("Target Velocity", curTargetVelocity);
        telemetry.addData("Current Velocity", curVelocity);
        telemetry.addData("Error", error);
        telemetry.addData("P", p);
        telemetry.addData("F", f);
        telemetry.addData("Step Size", stepSizes[stepIndex]);
        telemetry.update();
    }
}
