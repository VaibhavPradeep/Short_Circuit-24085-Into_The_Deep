package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.dfrobot.HuskyLens;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.internal.system.Deadline;

import java.util.concurrent.TimeUnit;

@Config
@TeleOp(name = "CR servo test")
public class CRServoTest extends OpMode {
    final int READ_PERIOD = 1;

    // TODO: find lever pos
    double leverPos = 0.5;
    DcMotor intakeMotor;
    Servo pitchServo;
    DcMotor rotationMotor;
    DcMotor shootingMotor;

    DcMotor transferMotor;
    CRServo sorterServo;
    Servo leverServo;

    ColorSensor colorSensor;
    HuskyLens huskyLens;
    Deadline rateLimit;
    public static double pos = 0;
    
    @Override
    public void init() {
        intakeMotor = hardwareMap.get(DcMotor.class, "intakeMotor");
        pitchServo = hardwareMap.get(Servo.class,"pitchServo");
        rotationMotor = hardwareMap.get(DcMotor.class, "rotationMotor");
        shootingMotor = hardwareMap.get(DcMotor.class, "shootingMotor");
        huskyLens = hardwareMap.get(HuskyLens.class, "huskyLens");

        transferMotor = hardwareMap.get(DcMotor.class, "transferMotor");
        sorterServo = hardwareMap.get(CRServo.class, "sorterServo");
        leverServo = hardwareMap.get(Servo.class,"leverServo");
        leverServo.setDirection(Servo.Direction.REVERSE);
        intakeMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        rotationMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rotationMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Rate limiter for telemetry
        rateLimit = new Deadline(READ_PERIOD, TimeUnit.SECONDS);
        rateLimit.expire();

        leverServo.setPosition(0);

        // Choose the algorithm
        huskyLens.selectAlgorithm(HuskyLens.Algorithm.TAG_RECOGNITION);
        // huskyLens.selectAlgorithm(HuskyLens.Algorithm.COLOR_RECOGNITION);
    }

    @Override
    public void loop() {

        sorterServo.setPower(1);
        //intakeMotor.setPower(1);
        //leverServo.setPosition(pos);
    }
}
