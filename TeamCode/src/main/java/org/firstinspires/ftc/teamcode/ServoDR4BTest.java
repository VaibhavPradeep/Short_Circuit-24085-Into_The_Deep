package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

@Config
@TeleOp(name = "Servo DR4B Test", group = "Testing")
public class ServoDR4BTest extends LinearOpMode {
    Servo Servo1;
    Servo Servo2;
    Servo Servo3;

    DcMotor leftDR4BMotor;
    DcMotor rightDR4BMotor;

    // Expose servo position to FTC Dashboard
    public static double servoPosition = 0.4;
    public static int DR4BPosition = 250;
    public static double clawPos = 0;

    public void DR4BMove(int ticks) {
        leftDR4BMotor.setTargetPosition(ticks);
        rightDR4BMotor.setTargetPosition(ticks);
        leftDR4BMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightDR4BMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftDR4BMotor.setPower(1);
        rightDR4BMotor.setPower(1);

        if (leftDR4BMotor.getCurrentPosition() == ticks && rightDR4BMotor.getCurrentPosition() == ticks) {
            leftDR4BMotor.setPower(0);
            rightDR4BMotor.setPower(0);
        }
    }

    @Override
    public void runOpMode() throws InterruptedException {

        // LEFT SERVO SHOULD BE REVERSED
        Servo1 = hardwareMap.get(Servo.class, "servo1");
        Servo2 = hardwareMap.get(Servo.class, "servo2");
        Servo3 = hardwareMap.get(Servo.class, "servo3");
        Servo3.setDirection(Servo.Direction.FORWARD);

        Servo2.setDirection(Servo.Direction.REVERSE);

        leftDR4BMotor = hardwareMap.get(DcMotor.class, "leftDR4BMotor");
        rightDR4BMotor = hardwareMap.get(DcMotor.class, "rightDR4BMotor");
        rightDR4BMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        leftDR4BMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightDR4BMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftDR4BMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightDR4BMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // Set initial positions
        Servo1.setPosition(0);
        Servo2.setPosition(0);

        waitForStart();

        while (opModeIsActive()) {
            // Update servo positions based on the variable values
            Servo1.setPosition(servoPosition);
            Servo2.setPosition(servoPosition);

            Servo3.setPosition(clawPos);

            DR4BMove(DR4BPosition);

            // Add telemetry for debugging
            telemetry.addData("Servo1 Position", servoPosition);
            telemetry.addData("Servo2 Position", servoPosition);
            telemetry.update();
        }
    }
}