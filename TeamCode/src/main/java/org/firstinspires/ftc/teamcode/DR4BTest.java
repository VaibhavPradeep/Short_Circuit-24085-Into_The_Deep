package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name="motor encoder test", group="Testing")
public class DR4BTest extends OpMode {
    private DcMotor motor1;
    private DcMotor motor2;

    public void DR4BMoving(double lefttrigger2, double righttrigger2) {
        int rightPos = motor2.getCurrentPosition();
        int leftPos = motor1.getCurrentPosition();

        double DR4BSpeed = 0;

        if(lefttrigger2 > 0.1) {
            rightPos += 100;
            leftPos += 100;
            DR4BSpeed = lefttrigger2;
        }
        else if(righttrigger2 > 0.1) {
            rightPos -= 100;
            leftPos -= 100;
            DR4BSpeed = righttrigger2;
        }

        motor1.setTargetPosition(leftPos);
        motor2.setTargetPosition(rightPos);
        motor1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motor2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motor1.setPower(DR4BSpeed);
        motor2.setPower(DR4BSpeed);

        while (motor1.isBusy() && motor2.isBusy()) {
            // Do nothing, just wait for the motors to finish moving
        }

        motor1.setPower(0);
        motor2.setPower(0);
        DR4BSpeed = 0;
    }

    @Override
    public void init() {
        // Initialize motors from the hardware map
        motor1 = hardwareMap.get(DcMotor.class, "motor1");
        motor2 = hardwareMap.get(DcMotor.class, "motor2");

        motor2.setDirection(DcMotorSimple.Direction.REVERSE);

        // Set motors to run using encoders, if needed
        motor1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        motor1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motor2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    @Override
    public void loop() {

        DR4BMoving(gamepad2.left_trigger, gamepad2.right_trigger);

        telemetry.addData("leftDR4B", motor1.getCurrentPosition());
        telemetry.addData("rightDR4B", motor2.getCurrentPosition());
        telemetry.update();

    }
}
