package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

@Config
@TeleOp(name = "CoaxialTesting", group = "Testing")
public class CoaxialTesting extends OpMode {

    Servo leftCV4BServo;
    Servo rightCV4BServo;
    Servo rotateIntakeServo;
    Servo wristServo;
    Servo intakeClawServo;

    public static double CV4BPos = 0;
    public static double RotateIntakePos = 0;
    public static double WristPos = 0;
    public static double IntakeClawPos = 0;

    @Override
    public void init() {
        leftCV4BServo = hardwareMap.get(Servo.class, "leftCV4BServo");
        rightCV4BServo = hardwareMap.get(Servo.class, "rightCV4BServo");
        rightCV4BServo.setDirection(Servo.Direction.REVERSE);
        rotateIntakeServo = hardwareMap.get(Servo.class, "rotateIntakeServo");
        wristServo = hardwareMap.get(Servo.class, "wristServo");
        intakeClawServo = hardwareMap.get(Servo.class, "intakeClawServo");
    }

    @Override
    public void loop() {
        leftCV4BServo.setPosition(CV4BPos);
        rightCV4BServo.setPosition(CV4BPos);
        rotateIntakeServo.setPosition(RotateIntakePos);
        wristServo.setPosition(WristPos);
        intakeClawServo.setPosition(IntakeClawPos);
    }

    // 0.2 cv4b, 0.4 rotate intake servo for getting in submersible
    // 0.255 for cv4b and  0.7 for rotate intake to get a thing from submersible
    // 0 for pick and 0.34 for other picking, wrist servo
    // all 0.12 for rotate intake for sagging, 0 for cv4b
    // 0 for everything not sagging
}
