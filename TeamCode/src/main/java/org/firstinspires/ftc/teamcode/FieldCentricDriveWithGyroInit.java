package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.hardware.bosch.BHI260IMU;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

@TeleOp(name = "Field Centric Drive with Gyro Init", group = "Linear Opmode")
public class FieldCentricDriveWithGyroInit extends LinearOpMode {

    private DcMotor leftFront;
    private DcMotor leftBack;
    private DcMotor rightFront;
    private DcMotor rightBack;
    private BHI260IMU imu;
    private BHI260IMU.Parameters parameters;
    private Orientation angles;

    @Override
    public void runOpMode() {
        leftFront = hardwareMap.get(DcMotor.class, "frontLeft");
        leftBack = hardwareMap.get(DcMotor.class, "backLeft");
        rightFront = hardwareMap.get(DcMotor.class, "frontRight");
        rightBack = hardwareMap.get(DcMotor.class, "backRight");

        leftFront.setDirection(DcMotor.Direction.REVERSE);
        leftBack.setDirection(DcMotor.Direction.REVERSE);
        rightFront.setDirection(DcMotor.Direction.FORWARD);
        rightBack.setDirection(DcMotor.Direction.FORWARD);

        BHI260IMU imu;
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

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            double y = -gamepad1.left_stick_y; // CHANGE NEG OR POS
            double x = gamepad1.left_stick_x * 1.1; // CHANGE NEG OR POS
            double rx = gamepad1.right_stick_x; // rotating

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

            leftFront.setPower(frontLeftPower);
            leftBack.setPower(backLeftPower);
            rightFront.setPower(frontRightPower);
            rightBack.setPower(backRightPower);

            telemetry.addData("Yaw (radians)", yaw);
            telemetry.addData("Pitch (radians)", pitch);
            telemetry.addData("Roll (radians)", roll);
            telemetry.addData("FL Power", frontLeftPower);
            telemetry.addData("BL Power", backLeftPower);
            telemetry.addData("FR Power", frontRightPower);
            telemetry.addData("BR Power", backRightPower);
            telemetry.update();
        }
    }
}
