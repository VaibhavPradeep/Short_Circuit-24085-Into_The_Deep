package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name = "TeleOpMeet3")
public class TeleOpMeet3 extends LinearOpMode {

    IntakeLiftCamera ILC = new IntakeLiftCamera();
    Drivetrain drivetrain = new Drivetrain(this);

    boolean aAlreadyPressed;
    boolean motorOn;

    public void driveMecanum(double left_y, double left_x, double right_x, double left_trigger){
        double maxPower = Math.max(Math.abs(left_y) + Math.abs(left_x) + Math.abs(right_x), 0.8);
        drivetrain.frontLeft.setPower((left_y - left_x - right_x) / maxPower);
        drivetrain.frontRight.setPower((left_y + left_x + right_x) / maxPower);
        drivetrain.backLeft.setPower((left_y + left_x - right_x) / maxPower);
        drivetrain.backRight.setPower((left_y - left_x + right_x) / maxPower);
    }

    @Override
    public void runOpMode() throws InterruptedException {
        drivetrain.initDrivetrain(hardwareMap);
        ILC.initIntakeLiftCamera(hardwareMap);

        ILC.leftCV4BServo.setPosition(0);
        ILC.rightCV4BServo.setPosition(0);
        ILC.rotateIntakeServo.setPosition(0);
        ILC.leftOutakeServo.setPosition(0);
        ILC.rightOutakeServo.setPosition(0);
        ILC.clawServo.setPosition(0.4);
        ILC.intakeClawServo.setPosition(0);

        waitForStart();

        while(opModeIsActive()) {
            telemetry.update();

            // drivetrain
            driveMecanum(gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x, gamepad1.left_trigger);

            // double reverse four bar
            // 215 dr4b, 0.5 outake bar

            // specimen chamber release and withdrawal
            /*
            if (gamepad2.left_trigger > 0.75) {
                ILC.DR4BMove(215);
                sleep(500); // Pauses the program for 500 milliseconds (1/2 second)
                ILC.depositOutake();
                sleep(10);
                ILC.collectBrickClaw();
            } else if (gamepad2.right_trigger > 0.75) {
                ILC.DR4BMove(-215);
                sleep(500); // Pauses the program for 500 milliseconds (1/2 second)
                ILC.standbyOutake();
                sleep(10);
                ILC.holdBrickClaw();
            }

             */

            if(gamepad2.dpad_up) {
                ILC.dPadMove("up");
            }
            else if (gamepad2.dpad_down) {
                ILC.dPadMove("down");
            }

            // intake
            if (gamepad1.left_trigger > 0.75) {
                ILC.holdBrickIntake();
            } else if (gamepad1.left_trigger > 0.75) {
                ILC.collectBrickIntake();
            }

            // coaxial virtual four bar
            if (gamepad1.dpad_up) {
                ILC.transferOrZeroCV4B();
            } else if (gamepad1.dpad_left) {
                ILC.submersibleCV4B();
            } else if (gamepad1.dpad_down) {
                ILC.collectCV4B();
            }

            // rotate intake
            if (gamepad1.x) {
                ILC.rotateToIntake();
            } else if (gamepad1.b) {
                ILC.rotateToTransfer();
            }

            // outake servo bars
            if (gamepad2.x) {
                ILC.standbyOutake();
            } else if (gamepad2.b) {
                ILC.specimenCollectOutake();
            } else if (gamepad2.y) {
                ILC.depositOutake();
            }

            // claw servo
            if (gamepad2.left_trigger > 0.75) {
                ILC.collectBrickClaw();
            } else if (gamepad2.right_trigger > 0.75) {
                ILC.holdBrickClaw();
            }

            // intake claw
            if (gamepad1.left_trigger > 0.75) {
                ILC.collectBrickIntake();
            } else if (gamepad1.right_trigger > 0.75) {
                ILC.holdBrickIntake();
            }


            if(gamepad2.a && !aAlreadyPressed) {
                motorOn = !motorOn;
                if (motorOn) {
                    ILC.standbyOutake();
                    ILC.DR4BMove(315);
                    ILC.depositOutake();
                    ILC.collectBrickClaw();
                } else {
                    ILC.standbyOutake();
                    ILC.DR4BMove(-315);
                }
            }
        }
    }
}
