package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "Servo Test", group="Testing")
public class ServosTest extends LinearOpMode {
    Servo Servo1;
    Servo Servo2;
    // LEFT SERVO SHOULD BE REVERSED

    @Override
    public void runOpMode() {
        Servo1 = hardwareMap.get(Servo.class, "servo1");
        Servo2 = hardwareMap.get(Servo.class, "servo2");

        Servo2.setDirection(Servo.Direction.REVERSE);

        Servo1.setPosition(0);
        Servo2.setPosition(0);

        waitForStart();

        while (opModeIsActive()) {
            if (gamepad1.y) {
                Servo1.setPosition(0);
                Servo2.setPosition(0);
            } else if (gamepad1.x) {
                Servo1.setPosition(0.07);
                Servo2.setPosition(0.07);
            } else if (gamepad1.a) {
                Servo1.setPosition(0.14);
                Servo2.setPosition(0.14);
            } else if (gamepad1.b) {
                Servo1.setPosition(0.28);
                Servo2.setPosition(0.28);
            } else if (gamepad1.dpad_up) {
                Servo1.setPosition(0.35);
                Servo2.setPosition(0.35);
            } else if (gamepad1.dpad_left) {
                Servo1.setPosition(0.42);
                Servo2.setPosition(0.42);
            } else if (gamepad1.dpad_down) {
                Servo1.setPosition(0.49);
                Servo2.setPosition(0.49);
            } else if (gamepad1.dpad_right) {
                Servo1.setPosition(0.56);
                Servo2.setPosition(0.56);
            }
        }
    }
}