package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;

@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name="Dual Motor Control with Bumpers", group="Testing")
public class DualMotorControlWithBumpers extends OpMode {

    private DcMotor motor1;
    private DcMotor motor2;

    @Override
    public void init() {
        // Initialize motors from the hardware map
        motor1 = hardwareMap.get(DcMotor.class, "motor1");
        motor2 = hardwareMap.get(DcMotor.class, "motor2");

        // Set motors to run using encoders, if needed
        motor1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motor2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    @Override
    public void loop() {
        double power = 0.5; // Adjust power level as needed, i

        // Control motor power based on bumper input
        if (gamepad1.left_bumper) {
            // Move motors forward
            motor1.setPower(power);
            motor2.setPower(power);
        } else if (gamepad1.right_bumper) {
            // Move motors backward
            motor1.setPower(-power);
            motor2.setPower(-power);
        } else {
            // Stop motors if no bumper is pressed
            motor1.setPower(0);
            motor2.setPower(0);
        }

        // Telemetry for monitoring
        telemetry.addData("Motor 1 Power", motor1.getPower());
        telemetry.addData("Motor 2 Power", motor2.getPower());
        telemetry.update();
    }
}
