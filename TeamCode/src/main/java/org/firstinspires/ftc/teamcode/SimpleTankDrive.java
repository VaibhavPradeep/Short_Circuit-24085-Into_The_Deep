package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp(name="Simple Tank Drive", group="Basic")
public class SimpleTankDrive extends OpMode {

    // 4A. Declare motor variables
    private DcMotor leftDrive;
    private DcMotor rightDrive;

    // 5A. init() runs once when INIT is pressed
    @Override
    public void init() {
        // 5B. Map hardware names to variables
        leftDrive = hardwareMap.get(DcMotor.class, "left_drive");
        rightDrive = hardwareMap.get(DcMotor.class, "right_drive");

        // 5C. Reverse right motor so both sides drive forward together
        leftDrive.setDirection(DcMotor.Direction.FORWARD);
        rightDrive.setDirection(DcMotor.Direction.REVERSE);
    }

    // 6A. loop() runs repeatedly after PLAY is pressed
    @Override
    public void loop() {
        // 6B. Read driver joystick inputs
        double leftPower = -gamepad1.left_stick_y;
        double rightPower = -gamepad1.right_stick_y;

        // 6C. Apply power to motors
        leftDrive.setPower(leftPower);
        rightDrive.setPower(rightPower);

        // 6D. Update telemetry on Driver Station for each motor
        telemetry.addData("Left Power", leftPower);
        telemetry.addData("Right Power", rightPower);
        telemetry.update();
    }
}

