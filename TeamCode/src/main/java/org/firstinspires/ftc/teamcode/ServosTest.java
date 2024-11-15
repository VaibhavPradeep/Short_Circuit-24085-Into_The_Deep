

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "servotest")
public class ServosTest extends OpMode {
    Servo Servo1;
    Servo Servo2;
    boolean aAlreadyPressed;
    boolean aServoOn;

    boolean bAlreadyPressed;
    boolean bServoOn;

    boolean xAlreadyPressed;
    boolean xServoOn;

    boolean leftAlreadyPressed;
    boolean leftServoOn;

    boolean rightAlreadyPressed;
    boolean rightServoOn;
    //oo


    public void moveServo(boolean leftbump2, boolean rightbump2) {
        double leftservoPosition = Servo1.getPosition();
        double rightservoPosition = Servo2.getPosition();

        if (leftbump2) {
            leftservoPosition += 0.01;
            rightservoPosition += 0.01;
            if (leftservoPosition > 1) {
                leftservoPosition = 1;
            }
            if (rightservoPosition > 1) {
                rightservoPosition = 1;
            }
        } else if (rightbump2) {
            leftservoPosition -= 0.01;
            rightservoPosition -= 0.01;
            if (leftservoPosition < 0) {
                leftservoPosition = 0;
            }
            if (rightservoPosition < 0) {
                rightservoPosition = 0;
            }
        }

        Servo1.setPosition(leftservoPosition);
        Servo2.setPosition(rightservoPosition);
        telemetry.addData("left servo pos", Servo1.getPosition());
        telemetry.addData("right servo pos", Servo2.getPosition());

        leftservoPosition = Servo1.getPosition();
        rightservoPosition = Servo2.getPosition();

    }

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

        moveServo(gamepad1.left_bumper, gamepad1.right_bumper);
        
        /* intake
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
        */
        
        



    }
}
