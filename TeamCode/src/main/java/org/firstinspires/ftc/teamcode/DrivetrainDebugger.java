package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp(name = "DrivetrainDebugger", group = "Testing")
public class DrivetrainDebugger extends LinearOpMode {

    private DcMotor leftFront;
    private DcMotor leftBack;
    private DcMotor rightFront;
    private DcMotor rightBack;

    double speed = 0.3;

    @Override
    public void runOpMode() throws InterruptedException {
        leftFront = hardwareMap.get(DcMotor.class, "leftFront");
        leftBack = hardwareMap.get(DcMotor.class, "leftBack");
        rightFront = hardwareMap.get(DcMotor.class, "rightFront");
        rightBack = hardwareMap.get(DcMotor.class, "rightBack");

        leftFront.setDirection(DcMotor.Direction.REVERSE);
        leftBack.setDirection(DcMotor.Direction.REVERSE);
        rightFront.setDirection(DcMotor.Direction.FORWARD);
        rightBack.setDirection(DcMotor.Direction.FORWARD);

        waitForStart();

        while (opModeIsActive()) {
            if (gamepad1.y) {
                leftFront.setPower(speed);
                leftBack.setPower(speed);
                rightFront.setPower(speed);
                rightBack.setPower(speed);
            } else if (gamepad1.a) {
                leftFront.setPower(-speed);
                leftBack.setPower(-speed);
                rightFront.setPower(-speed);
                rightBack.setPower(-speed);
            } else if (gamepad1.x) {
                leftFront.setPower(-speed);
                leftBack.setPower(speed);
                rightFront.setPower(speed);
                rightBack.setPower(-speed);
            } else if (gamepad1.b) {
                leftFront.setPower(speed);
                leftBack.setPower(-speed);
                rightFront.setPower(-speed);
                rightBack.setPower(speed);
            } else if (gamepad1.dpad_left) {
                leftFront.setPower(-speed);
                leftBack.setPower(-speed);
                rightFront.setPower(speed);
                rightBack.setPower(speed);
            }  else if (gamepad1.dpad_right) {
                leftFront.setPower(speed);
                leftBack.setPower(speed);
                rightFront.setPower(-speed);
                rightBack.setPower(-speed);
            } else if (gamepad1.left_bumper) {
                leftFront.setPower(0);
                leftBack.setPower(0);
                rightFront.setPower(0);
                rightBack.setPower(0);
            }
        }
    }
}
