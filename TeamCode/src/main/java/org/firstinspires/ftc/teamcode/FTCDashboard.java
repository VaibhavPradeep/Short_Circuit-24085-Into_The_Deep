package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "RobotTest")
@Config
public class FTCDashboard extends LinearOpMode {

    Servo Servo1;
    Servo Servo2;
    public static double armPos = 0;

    @Override public void runOpMode() throws InterruptedException {

        // Wait until we're told to go
        waitForStart();
        Servo1 = hardwareMap.get(Servo.class, "servo1");
        Servo2 = hardwareMap.get(Servo.class, "servo1");

        while (opModeIsActive()) {
            Servo1.setPosition(armPos);
            Servo2.setPosition(armPos);
        }
    }
}




/*
package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Servo;
import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;

@Config
@TeleOp(name = "Servo Tester", group = "Testing")
public class FTCDashboard extends LinearOpMode {

    // Declare the servo
    public static String servoName = "Servo1"; // Name of the servo in the config
    public static double servoPosition = 0;    // Initial servo position (range 0 to 1)

    private Servo Servo1;

    @Override
    public void runOpMode() {
        // Initialize servo
        Servo1 = hardwareMap.get(Servo.class, servoName);

        // Initialize FTC Dashboard
        FtcDashboard dashboard = FtcDashboard.getInstance();

        telemetry.addLine("Ready to start");
        telemetry.update();

        // Wait for start command
        waitForStart();

        while (opModeIsActive()) {
            // Set servo position
            Servo1.setPosition(servoPosition);

            // Send telemetry to dashboard
            telemetry.addData("Servo Name", servoName);
            telemetry.addData("Servo Position", servoPosition);
            telemetry.update();
        }
    }
}

 */