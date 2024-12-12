package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@TeleOp(name = "DR4B Test")
public class DR4BTest extends OpMode {
    int[] maxPositions = {4500, 4500}; // potentially 4600

    int[] minPositions = {0, 0};
    
    private DcMotor motor1;
    private DcMotor motor2;

    public void dPadMove(String direction) {
        int rightPos = motor2.getCurrentPosition();
        int leftPos = motor1.getCurrentPosition();

        if(direction.equals("up") && rightPos <= maxPositions[0] && leftPos <= maxPositions[1]) {
            rightPos += 100;
            leftPos += 100;
        }
        else if(direction.equals("down") && rightPos >= minPositions[0] && leftPos >= minPositions[1]) {
            rightPos -= 100;
            leftPos -= 100;
        }

        motor1.setTargetPosition(leftPos);
        motor2.setTargetPosition(rightPos);
        motor1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motor2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motor1.setPower(1);
        motor2.setPower(1);

        if (motor1.getCurrentPosition() == leftPos && motor2.getCurrentPosition() == rightPos) {
            motor1.setPower(0);
            motor2.setPower(0);
        }
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

        if(gamepad1.dpad_up) {
            dPadMove("up");
        }
        else if (gamepad1.dpad_down) {
            dPadMove("down");
        }

        telemetry.addData("leftDR4B", motor1.getCurrentPosition());
        telemetry.addData("rightDR4B", motor2.getCurrentPosition());
        telemetry.update();

    }
}
