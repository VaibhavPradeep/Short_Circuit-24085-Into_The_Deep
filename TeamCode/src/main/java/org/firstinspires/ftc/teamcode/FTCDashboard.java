package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

@Config
@TeleOp(name = "FTC Dashboard Decode")
public class FTCDashboard extends OpMode {

    DcMotor intakeMotor;
    Servo pitchServo;
    DcMotor rotationMotor;
    DcMotor shootingMotor;

    DcMotor transferMotor;
    CRServo sorterServo;
    Servo leverServo;

    public static double leverServoPos = 0;
    public static double intakeMotorPower = 0;
    public static double rotationMotorPower = 0;
    public static double shootingMotorPower = 0;
    public static double transferMotorPower = 0;
    public static double sorterServoPower = 0;



    @Override
    public void init() {
        intakeMotor = hardwareMap.get(DcMotor.class, "intakeMotor");
        rotationMotor = hardwareMap.get(DcMotor.class, "rotationMotor");
        shootingMotor = hardwareMap.get(DcMotor.class, "shootingMotor");
        pitchServo = hardwareMap.get(Servo.class,"pitchServo");

        transferMotor = hardwareMap.get(DcMotor.class, "transferMotor");
        sorterServo = hardwareMap.get(CRServo.class, "sorterServo");
        leverServo = hardwareMap.get(Servo.class,"leverServo");

        rotationMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rotationMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    @Override
    public void loop() {
        leverServo.setPosition(leverServoPos);
        intakeMotor.setPower(intakeMotorPower);
        rotationMotor.setPower(rotationMotorPower);
        shootingMotor.setPower(shootingMotorPower);
        transferMotor.setPower(transferMotorPower);
        sorterServo.setPower(sorterServoPower);
    }
}
