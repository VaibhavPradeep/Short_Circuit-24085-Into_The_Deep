package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

public class DriveTrainYawMethods {

    final double WHEEL_DIAMETER = 4.09;
    final double TICKS_PER_WHEEL_REVOLUTION = 384.5;
    final double INCHES_PER_WHEEL_REVOLUTION = 12.8632;
    final double WHEEL_COUNTS_PER_INCH = (TICKS_PER_WHEEL_REVOLUTION) / (WHEEL_DIAMETER * Math.PI);
    final double DRIVE_GEAR_REDUCTION = 1.0; // just one so not needed in equation

    DcMotor frontLeft;
    DcMotor frontRight;
    DcMotor backLeft;
    DcMotor backRight;

    static private final int Reverse = 1;
    static private final int Forward = -1;

    private ElapsedTime runtime = new ElapsedTime();

    IMU imu;
    IMU.Parameters parameters;
    YawPitchRollAngles angles;

    public void initDrivetrainYaw(HardwareMap hwMap) {
        frontLeft = hwMap.get(DcMotor.class, "frontLeft");
        backLeft = hwMap.get(DcMotor.class, "backLeft");
        frontRight = hwMap.get(DcMotor.class, "frontRight");
        backRight = hwMap.get(DcMotor.class, "backRight");

        frontRight.setDirection(DcMotorSimple.Direction.REVERSE);
        backRight.setDirection(DcMotorSimple.Direction.REVERSE);

        frontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        frontLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        frontRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        frontLeft.setPower(0);
        backLeft.setPower(0);
        frontRight.setPower(0);
        backRight.setPower(0);
    }

    public void drive(double speed, double distance) {
        int newFrontLeftTargetDrive;
        int newBackLeftTargetDrive;
        int newFrontRightTargetDrive;
        int newBackRightTargetDrive;

        newFrontLeftTargetDrive = frontLeft.getCurrentPosition() - (int)(distance * WHEEL_COUNTS_PER_INCH);
        newBackLeftTargetDrive = backLeft.getCurrentPosition() - (int)(distance * WHEEL_COUNTS_PER_INCH);
        newFrontRightTargetDrive = frontRight.getCurrentPosition() - (int)(distance * WHEEL_COUNTS_PER_INCH);
        newBackRightTargetDrive = backRight.getCurrentPosition() - (int)(distance * WHEEL_COUNTS_PER_INCH);

        frontLeft.setTargetPosition(newFrontLeftTargetDrive);
        backLeft.setTargetPosition(newBackLeftTargetDrive);
        frontRight.setTargetPosition(newFrontRightTargetDrive);
        backRight.setTargetPosition(newBackRightTargetDrive);

        frontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        frontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        // reset the timeout time and start motion.
        runtime.reset();
        frontLeft.setPower(speed);
        backLeft.setPower(speed);
        frontRight.setPower(speed);
        backRight.setPower(speed);

        if (frontLeft.getCurrentPosition() == newFrontLeftTargetDrive && frontRight.getCurrentPosition() == newFrontRightTargetDrive && backLeft.getCurrentPosition() == newBackLeftTargetDrive && backRight.getCurrentPosition() == newBackRightTargetDrive) {
            frontLeft.setPower(0);
            backLeft.setPower(0);
            frontRight.setPower(0);
            backRight.setPower(0);
        }

    }

    public void strafe(String direction, double speed, double distance) {
        int newFrontLeftTargetStrafe;
        int newBackLeftTargetStrafe;
        int newFrontRightTargetSrafe;
        int newBackRightTargetStrafe;

        direction = direction.toLowerCase();

        if(direction.equals("right")) {
            newFrontLeftTargetStrafe = frontLeft.getCurrentPosition() - (int) (distance * WHEEL_COUNTS_PER_INCH);
            newBackLeftTargetStrafe = backLeft.getCurrentPosition() + (int) (distance * WHEEL_COUNTS_PER_INCH);
            newFrontRightTargetSrafe = frontRight.getCurrentPosition() + (int) (distance * WHEEL_COUNTS_PER_INCH);
            newBackRightTargetStrafe = backRight.getCurrentPosition() - (int) (distance * WHEEL_COUNTS_PER_INCH);
        }
        else if(direction.equals("left")) {
            newFrontLeftTargetStrafe = frontLeft.getCurrentPosition() + (int) (distance * WHEEL_COUNTS_PER_INCH);
            newBackLeftTargetStrafe = backLeft.getCurrentPosition() - (int) (distance * WHEEL_COUNTS_PER_INCH);
            newFrontRightTargetSrafe = frontRight.getCurrentPosition() - (int) (distance * WHEEL_COUNTS_PER_INCH);
            newBackRightTargetStrafe = backRight.getCurrentPosition() + (int) (distance * WHEEL_COUNTS_PER_INCH);
        }
        else {
            newFrontLeftTargetStrafe = frontLeft.getCurrentPosition();
            newBackLeftTargetStrafe = backLeft.getCurrentPosition();
            newFrontRightTargetSrafe = frontRight.getCurrentPosition();
            newBackRightTargetStrafe = backRight.getCurrentPosition();
        }

        frontLeft.setTargetPosition(newFrontLeftTargetStrafe);
        backLeft.setTargetPosition(newBackLeftTargetStrafe);
        frontRight.setTargetPosition(newFrontRightTargetSrafe);
        backRight.setTargetPosition(newBackRightTargetStrafe);

        frontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        frontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        // reset the timeout time and start motion.
        runtime.reset();
        frontLeft.setPower(Math.abs(speed));
        backLeft.setPower(Math.abs(speed));
        frontRight.setPower(Math.abs(speed));
        backRight.setPower(Math.abs(speed));

        if (frontLeft.getCurrentPosition() == newFrontLeftTargetStrafe && frontRight.getCurrentPosition() == newFrontRightTargetSrafe && backLeft.getCurrentPosition() == newBackLeftTargetStrafe && backRight.getCurrentPosition() == newBackRightTargetStrafe) {
            frontLeft.setPower(0);
            backLeft.setPower(0);
            frontRight.setPower(0);
            backRight.setPower(0);
        }
    }

    public double getAngleDifference(double angle1, double angle2) {

        return (angle1 - angle2 + 540) % 360 - 180;

    }

    public void turn(double power, double angle, String direction) {
        angles = imu.getRobotYawPitchRollAngles();

        frontLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        frontRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        int directionMultiplier = -1;

        if (direction.equals("closest")) {

            if (getAngleDifference(angles.getYaw(AngleUnit.DEGREES), angle) < 0) {
                power *= -1;
                directionMultiplier *= -1;
            }

        } else if (direction.equals("left")) {
            power *= -1;
            directionMultiplier *= -1;
        }

        frontLeft.setPower(power);
        backLeft.setPower(power);
        frontRight.setPower(-power);
        backRight.setPower(-power);

        while (frontLeft.isBusy() && frontRight.isBusy() && backLeft.isBusy() && backRight.isBusy() && directionMultiplier * getAngleDifference(angles.getYaw(AngleUnit.DEGREES), angle) < 0) {
            angles = imu.getRobotYawPitchRollAngles();
        }

        frontLeft.setPower(0);
        backLeft.setPower(0);
        frontRight.setPower(0);
        backRight.setPower(0);

        frontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        frontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

    }

    public void initGyro(HardwareMap hwMap) {
        parameters = new IMU.Parameters(
                new RevHubOrientationOnRobot(
                        RevHubOrientationOnRobot.LogoFacingDirection.UP,
                        RevHubOrientationOnRobot.UsbFacingDirection.LEFT
                )
        );

        imu = hwMap.get(IMU.class, "imu");
        angles = imu.getRobotYawPitchRollAngles();
        imu.initialize(parameters);
    }

}
