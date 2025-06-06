package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BHI260IMU;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

@TeleOp(name = "TeleOp League Champs")
public class TeleOpLeagueChamps extends LinearOpMode {

    IntakeLiftCamera ILC = new IntakeLiftCamera();
    Drivetrain drivetrain = new Drivetrain(this);

    boolean aAlreadyPressed;
    boolean motorOn;

    ElapsedTime timer = new ElapsedTime();

    public void driveMecanum(double left_y, double left_x, double right_x, boolean brake){
        double maxPower = Math.max(Math.abs(left_y) + Math.abs(left_x) + Math.abs(right_x), 1);


        if (brake == true) {
            drivetrain.frontLeft.setPower(((left_y - left_x - right_x) / maxPower)/1.5);
            drivetrain.frontRight.setPower(((left_y + left_x + right_x) / maxPower));
            drivetrain.backLeft.setPower(((left_y + left_x - right_x) / maxPower)/1.5);
            drivetrain.backRight.setPower(((left_y - left_x + right_x) / maxPower)/1.5);
        } else {
            drivetrain.frontLeft.setPower((left_y - left_x - right_x) / maxPower);
            drivetrain.frontRight.setPower((left_y + left_x + right_x) / maxPower);
            drivetrain.backLeft.setPower((left_y + left_x - right_x) / maxPower);
            drivetrain.backRight.setPower((left_y - left_x + right_x) / maxPower);
        }

        /*
        drivetrain.frontLeft.setPower((left_y - left_x - right_x) / maxPower);
        drivetrain.frontRight.setPower((left_y + left_x + right_x) / maxPower);
        drivetrain.backLeft.setPower((left_y + left_x - right_x) / maxPower);
        drivetrain.backRight.setPower((left_y - left_x + right_x) / maxPower);
         */
    }



    @Override
    public void runOpMode() throws InterruptedException {
        drivetrain.initDrivetrain(hardwareMap);
        ILC.initIntakeLiftCamera(hardwareMap);

        BHI260IMU imu;
        BHI260IMU.Parameters parameters;
        YawPitchRollAngles angles;

        parameters = new BHI260IMU.Parameters(
                new RevHubOrientationOnRobot(
                        RevHubOrientationOnRobot.LogoFacingDirection.UP,
                        RevHubOrientationOnRobot.UsbFacingDirection.FORWARD
                )
        );

        imu = hardwareMap.get(BHI260IMU.class, "imu");
        imu.initialize(parameters);

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();

        while(opModeIsActive()) {
            telemetry.update();

            // drivetrain
            driveMecanum(gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x, gamepad1.left_bumper);

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
            /*
            double y = -gamepad1.left_stick_y; // CHANGE NEG OR POS
            double x = -gamepad1.left_stick_x * 1.1; // CHANGE NEG OR POS
            double rx = -gamepad1.right_stick_x; // rotating

            angles = imu.getRobotYawPitchRollAngles();
            double yaw = angles.getYaw(AngleUnit.RADIANS); // robot heading
            double pitch = angles.getPitch(AngleUnit.RADIANS);
            double roll = angles.getRoll(AngleUnit.RADIANS);

            double rotatedX = x * Math.cos(yaw) - y * Math.sin(yaw);
            double rotatedY = x * Math.sin(yaw) + y * Math.cos(yaw);

            double denominator = Math.max(Math.abs(rotatedY) + Math.abs(rotatedX) + Math.abs(rx), 1.0);
            double frontLeftPower = (rotatedY + rotatedX + rx) / denominator;
            double backLeftPower = (rotatedY - rotatedX + rx) / denominator;
            double frontRightPower = (rotatedY - rotatedX - rx) / denominator;
            double backRightPower = (rotatedY + rotatedX - rx) / denominator;

            drivetrain.frontLeft.setPower(frontLeftPower);
            drivetrain.backLeft.setPower(backLeftPower);
            drivetrain.frontRight.setPower(frontRightPower);
            drivetrain.backRight.setPower(backRightPower);

            telemetry.addData("Yaw (radians)", yaw);
            telemetry.addData("Pitch (radians)", pitch);
            telemetry.addData("Roll (radians)", roll);
            telemetry.addData("FL Power", frontLeftPower);
            telemetry.addData("BL Power", backLeftPower);
            telemetry.addData("FR Power", frontRightPower);
            telemetry.addData("BR Power", backRightPower);
            telemetry.update();

             */

            if(gamepad2.dpad_up) {
                ILC.dPadMove("up");
            }
            else if (gamepad2.dpad_down) {
                ILC.dPadMove("down");
            }

            if (gamepad1.y) {
                ILC.normalPickup();
            } else if (gamepad1.a) {
                ILC.turnedPickup();
            }


            // coaxial virtual four bar
            if (gamepad1.dpad_up) {
                ILC.sagggingCV4B();
            } else if (gamepad1.dpad_right) {
                ILC.submersibleCV4B();
            } else if (gamepad1.dpad_down) {
                ILC.collectBrickIntake();
                ILC.collectCV4B();
            } else if (gamepad1.dpad_left) {
                ILC.zeroCV4B();
            }

            // outake servo bars
            if (gamepad2.x) {
                ILC.standbyOutake();
            } else if (gamepad2.b) {
                ILC.specimenCollectOutake();
            } else if (gamepad2.y) {
                ILC.depositOutake();
            }

            if (gamepad1.x) {
                ILC.holdBrickIntake();
            } else if (gamepad1.b) {
                ILC.collectBrickIntake();
            }

            // claw servo
            if (gamepad2.left_trigger > 0.75) {
                ILC.collectBrickClaw();
            } else if (gamepad2.right_trigger > 0.75) {
                ILC.holdBrickClaw();
            }


            if (gamepad1.left_trigger > 0.75) {
                /*
                timer.reset();
                ILC.holdBrickIntake();
                ILC.zeroCV4B();
                ILC.sagggingCV4B();
                ILC.standbyOutake();
                ILC.collectBrickClaw();

                 */
            } else if (gamepad1.right_trigger > 0.75) {
                //ILC.holdBrickClaw();
                //ILC.collectBrickIntake();
            }




            /*
            if(gamepad2.a && !aAlreadyPressed) {
                motorOn = !motorOn;
                if (motorOn) {
                    ILC.DR4BMove(850);
                    ILC.depositOutake();
                } else {
                    ILC.DR4BMove(-850);
                }

            }

             */
        }
    }
}
