package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp(name = "motortest")
public class MotorTest extends OpMode {
    DcMotor motor;

    @Override
    public void init() {
        motor = hardwareMap.get(DcMotor.class,"motor");
    }

    @Override
    public void loop() {
        motor.setPower(1);
    }
}
