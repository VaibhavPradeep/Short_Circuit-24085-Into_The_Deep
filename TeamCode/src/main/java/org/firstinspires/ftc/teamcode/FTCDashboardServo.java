package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;
import com.acmerobotics.dashboard.config.Config;

@Config
@TeleOp(name = "FTC Dashboard Servo")
public class FTCDashboardServo extends LinearOpMode {
    Servo Servo1;

    Servo Servo2;

    // Expose servo position to FTC Dashboard
    public static double servoPosition = 0.0;

    @Override
    public void runOpMode() throws InterruptedException {

        // LEFT SERVO SHOULD BE REVERSED
        Servo1 = hardwareMap.get(Servo.class, "servo1");
        Servo2 = hardwareMap.get(Servo.class, "servo2");

        Servo2.setDirection(Servo.Direction.REVERSE);

        // Set initial positions
        Servo1.setPosition(0);
        Servo2.setPosition(0);

        waitForStart();

        while (opModeIsActive()) {
            // Update servo positions based on the variable values
            Servo1.setPosition(servoPosition);
            Servo2.setPosition(servoPosition);

            // Add telemetry for debugging
            telemetry.addData("Servo1 Position", servoPosition);
            telemetry.addData("Servo2 Position", servoPosition);
            telemetry.update();
        }
    }
}