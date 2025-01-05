package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name = "TeleOpMeet3")
public class TeleOpMeet3 extends LinearOpMode {

    IntakeLiftCamera ILC = new IntakeLiftCamera();
    Drivetrain drivetrain = new Drivetrain(this);

    public void driveMecanum(double left_y, double left_x, double right_x, double left_trigger){
        double maxPower = Math.max(Math.abs(left_y) + Math.abs(left_x) + Math.abs(right_x), 1);
        if (left_trigger > 0.75) {
            maxPower = Math.max(Math.abs(left_y) + Math.abs(left_x) + Math.abs(right_x), 0.5);
        }
        drivetrain.frontLeft.setPower((left_y - left_x - right_x) / maxPower);
        drivetrain.frontRight.setPower((left_y + left_x + right_x) / maxPower);
        drivetrain.backLeft.setPower((left_y + left_x - right_x) / maxPower);
        drivetrain.backRight.setPower((left_y - left_x + right_x) / maxPower);
    }

    @Override
    public void runOpMode() throws InterruptedException {
        drivetrain.initDrivetrain(hardwareMap);
        ILC.initIntakeLiftCamera(hardwareMap);

        waitForStart();

        while(opModeIsActive()) {
            telemetry.update();

            // drivetrain
            driveMecanum(gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x, gamepad1.left_trigger);

            // double reverse four bar
            if(gamepad1.dpad_up) {
                ILC.dPadMove("up");
            }
            else if (gamepad1.dpad_down) {
                ILC.dPadMove("down");
            }

            // intake
            if (gamepad1.a) {
                ILC.intakeIn();
            } else if (gamepad1.x) {
                ILC.intakeOut();
            } else if (gamepad1.b) {
                ILC.intakeOff();
            }

            // coaxial virtual four bar
            if (gamepad2.dpad_up) {
                ILC.transferOrZeroCV4B();
            } else if (gamepad2.dpad_left) {
                ILC.submersibleCV4B();
            } else if (gamepad2.dpad_down) {
                ILC.collectCV4B();
            }

            // rotate intake
            if (gamepad2.a) {
                ILC.rotateToIntake();
            } else if (gamepad2.y) {
                ILC.rotateToTransfer();
            }

            // outake servo bars
            if (gamepad2.x) {
                ILC.transferOutake();
            } else if (gamepad2.b) {
                ILC.depositOutake();
            }

            // claw servo
            if (gamepad2.left_bumper) {
                ILC.collectBrick();
            } else if (gamepad2.right_bumper) {
                ILC.holdBrick();
            }
        }
    }
}
