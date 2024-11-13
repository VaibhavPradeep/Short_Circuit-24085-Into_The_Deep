

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "armservo")
public class ZeroServos extends OpMode {
    Servo Servo1;

    @Override
    public void init() {

        Servo1 = hardwareMap.get(Servo.class, "servo");

        Servo1.setPosition(0);

    }

    @Override
    public void loop() {
        if(gamepad1.a) {
            Servo1.setPosition(0);

        }
    }
}
