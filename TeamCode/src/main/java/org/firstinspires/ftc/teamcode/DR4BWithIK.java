package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@TeleOp(name = "DR4BWithIK", group = "Testing")
public class DR4BWithIK extends LinearOpMode {

    private DcMotor motor1;
    private DcMotor motor2;

    private static final double LINK_LENGTH = 25.3;
    private static final double BASE_HEIGHT = 28.2;
    private static final double ENCODER_TICKS_PER_REV = 537.6;
    private static final double GEAR_RATIO = 3.0;
    private static final double MOTOR_SPOOL_RADIUS = 1.5;

    @Override
    public void runOpMode() throws InterruptedException {

        motor1 = hardwareMap.get(DcMotor.class, "motor1");
        motor2 = hardwareMap.get(DcMotor.class, "motor2");

        motor1.setDirection(DcMotorSimple.Direction.FORWARD);
        motor2.setDirection(DcMotorSimple.Direction.REVERSE);

        motor1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motor2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        waitForStart();

        double targetHeight = 1;
        int targetPosition = calculateMotorPosition(targetHeight);

        // Set the target position for both motors
        motor1.setTargetPosition(targetPosition);
        motor2.setTargetPosition(targetPosition);

        // Set motors to RUN_TO_POSITION mode
        motor1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motor2.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        motor1.setPower(0.3);
        motor2.setPower(0.3);

        while (opModeIsActive() && (motor1.isBusy() || motor2.isBusy())) {
            telemetry.addData("Target Height (cm)", targetHeight);
            telemetry.addData("Left Motor Position", motor1.getCurrentPosition());
            telemetry.addData("Right Motor Position", motor2.getCurrentPosition());
            telemetry.update();
        }

        // Stop motors once the target is reached
        motor1.setPower(0);
        motor2.setPower(0);
    }

    private int calculateMotorPosition(double targetHeight) {
        double heightDelta = targetHeight - BASE_HEIGHT;
        double stringDelta = 2 * Math.sqrt((LINK_LENGTH * LINK_LENGTH) + (heightDelta * heightDelta)) - 2 * LINK_LENGTH;
        double motorRotations = stringDelta / (2 * Math.PI * MOTOR_SPOOL_RADIUS);
        return (int) (motorRotations * ENCODER_TICKS_PER_REV * GEAR_RATIO);
    }
}

/*
Instructions for Robot-Specific Measurements
LINK_LENGTH: 25.3 cm

Measure the distance between the pivot points of the bars (joint-to-joint distance).
Units: Centimeters (cm).
BASE_HEIGHT: 28.2 cm

Measure the vertical distance from the ground (or mounting base) to the bottom pivot joint of the DR4B.
Units: Centimeters (cm).
ENCODER_TICKS_PER_REV: 537.6

Check your motor specifications for the number of encoder ticks per revolution.
Common values: 1440 (Tetrix), 384.5 (GoBilda).
GEAR_RATIO: 3:1 so 3.0

If your lift has gearing, input the gear ratio as output/input.
        Example: A 5:1 gear ratio = 5.0.
        MOTOR_SPOOL_RADIUS: 1.479 cm

Measure the radius of the spool or pulley that winds/unwinds the lift string.
Units: Centimeters (cm).
targetHeight: 46.7 cm

Input the desired vertical extension for the lift.
Units: Centimeters (cm).
 */
