package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BHI260IMU;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

@TeleOp(name = "TeleOpITD")
public class TeleOpFC extends LinearOpMode {
    IntakeLiftCamera ILC = new IntakeLiftCamera();
    Drivetrain drivetrain = new Drivetrain(this);

    int[] maxPositions = {4500, 4500}; // potentially 4600

    int[] minPositions = {0, 0};

    @Override
    public void runOpMode() throws InterruptedException {
        ILC.initIntakeLiftCamera(hardwareMap);
        drivetrain.initDrivetrain(hardwareMap);

        double speed = 0.8;

        BHI260IMU imu; // Updated to use the new IMU type
        BHI260IMU.Parameters parameters;
        YawPitchRollAngles angles;

        parameters = new BHI260IMU.Parameters(
                new RevHubOrientationOnRobot(
                        RevHubOrientationOnRobot.LogoFacingDirection.UP,
                        RevHubOrientationOnRobot.UsbFacingDirection.LEFT
                )
        );


        imu = hardwareMap.get(BHI260IMU.class, "imu");
        imu.initialize(parameters);

        // Wait until we're told to go
        waitForStart();

        while (opModeIsActive()) {
            telemetry.update();

            // Drivetrain
            angles = imu.getRobotYawPitchRollAngles();
            double r = Math.hypot(gamepad1.left_stick_x, gamepad1.left_stick_y);
            double driveAngle = Math.atan2(-gamepad1.left_stick_y, gamepad1.left_stick_x) - Math.PI / 4 - angles.getYaw(AngleUnit.RADIANS);
            double rightX = gamepad1.right_stick_x;

            if (gamepad1.right_trigger > 0.75) {
                driveAngle = Math.atan2(gamepad1.left_stick_y, -gamepad1.left_stick_x) - Math.PI / 4;
            }

            final double FLPower = speed * (r * Math.cos(driveAngle) + rightX);
            final double BLPower = speed * (r * Math.sin(driveAngle) + rightX);
            final double FRPower = speed * (r * Math.sin(driveAngle) - rightX);
            final double BRPower = speed * (r * Math.cos(driveAngle) - rightX);

            drivetrain.frontLeft.setPower(FLPower);
            drivetrain.frontRight.setPower(FRPower);
            drivetrain.backLeft.setPower(BLPower);
            drivetrain.backRight.setPower(BRPower);
// Stopping robot from penetration
            if (gamepad1.right_trigger > 0.75) {
                speed = 0.4;

                drivetrain.frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                drivetrain.frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                drivetrain.backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                drivetrain.backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            } else {
                speed = 1;

                drivetrain.frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
                drivetrain.frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
                drivetrain.backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
                drivetrain.backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
            }

            // Double Reverse Four Bar
            int rightPos = ILC.rightDR4BMotor.getCurrentPosition();
            int leftPos = ILC.leftDR4BMotor.getCurrentPosition();

            if(gamepad2.dpad_up && rightPos <= maxPositions[0] && leftPos <= maxPositions[1]) {
                // Move motors forward
                ILC.leftDR4BMotor.setPower(1);
                ILC.rightDR4BMotor.setPower(1);
            }
            else if(gamepad2.dpad_down && rightPos >= minPositions[0] && leftPos >= minPositions[1]) {
                // Move motors backward
                ILC.leftDR4BMotor.setPower(-1);
                ILC.rightDR4BMotor.setPower(-1);
            } else {
                // Stop motors if no bumper is presseduughbg
                ILC.leftDR4BMotor.setPower(0);
                ILC.rightDR4BMotor.setPower(0);
            }

            // Intake
            if (gamepad1.a) {
                ILC.intakeIn();
            } else if (gamepad1.x) {
                ILC.intakeOut();
            } else if (gamepad1.b) {
                ILC.intakeOff();
            }

            // Coaxial Virtual Four Bar
            // Virtual Four Bar
            if (gamepad2.y) {
                ILC.transferOrZeroCV4B();
            } else if (gamepad2.a) {
                ILC.collectCV4B();
            }

            // Rotate Intake
            if (gamepad2.x) {
                ILC.rotateToTransfer();
            } else if (gamepad2.b) {
                ILC.rotateToIntake();
            }

            // Outake
            if (gamepad2.left_bumper) {
                ILC.depositOutake();
            } else if (gamepad1.right_bumper) {
                ILC.transferOrZeroOutake();
            }

            // Specimen
            if (gamepad1.left_bumper) {
                ILC.collectSpecimen();
            } else if (gamepad1.right_bumper) {
                ILC.holdSpecimen();
            }

            if (gamepad1.dpad_left) {
                ILC.startingArmOutake();
            } else if (gamepad1.dpad_right) {
                ILC.secondArmOutake();
            }

            ILC.addTelemetry(telemetry);
            telemetry.update();

            // sensors
            // 1.
        }
    }
}
