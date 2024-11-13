

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "servotest")
public class ZeroServos extends OpMode {
    Servo Servo1;
    boolean aAlreadyPressed;
    boolean aServoOn;

    boolean bAlreadyPressed;
    boolean bServoOn;

    boolean xAlreadyPressed;
    boolean xServoOn;

    @Override
    public void init() {

        Servo1 = hardwareMap.get(Servo.class, "servo");

        Servo1.setPosition(0);

    }

    @Override
    public void loop() {

        // intake
        if(gamepad1.a && !aAlreadyPressed){
            aServoOn = !aServoOn;
            if (aServoOn) {
                Servo1.setPosition(0);
            }
        }

        // to get over the submersible wall
        if(gamepad1.b && !bAlreadyPressed){
            bServoOn = !bServoOn;
            if (bServoOn) {
                Servo1.setPosition(0.15);
            }
        }

        // deposit
        if(gamepad1.x && !xAlreadyPressed){
            xServoOn = !xServoOn;
            if (xServoOn) {
                Servo1.setPosition(0.67);
            }
        }



    }
}
