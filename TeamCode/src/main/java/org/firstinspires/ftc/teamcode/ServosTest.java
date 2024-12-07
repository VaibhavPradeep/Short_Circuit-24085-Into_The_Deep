

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "servotest")
public class ServosTest extends OpMode {
    Servo Servo1;
    Servo Servo2;

    @Override
    public void init() {

        Servo1 = hardwareMap.get(Servo.class, "servo1");
        Servo2 = hardwareMap.get(Servo.class, "servo2");

        Servo2.setDirection(Servo.Direction.REVERSE);

        Servo1.setPosition(0);
        Servo2.setPosition(0);

    }

    @Override
    public void loop() {
        if (gamepad1.y) {
            Servo1.setPosition(0);
            Servo1.setPosition(0);
        } else if (gamepad1.x) {
            Servo1.setPosition(0.1);
            Servo1.setPosition(0.1);
        } else if (gamepad1.a) {
            Servo1.setPosition(0.2);
            Servo1.setPosition(0.2);
        } else if (gamepad1.b) {
            Servo1.setPosition(0.3);
            Servo1.setPosition(0.3);
        }
// o
    }
}
