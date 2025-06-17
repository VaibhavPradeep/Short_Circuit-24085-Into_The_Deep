package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "OutreachBotTeleOp")
public class OutreachBotTeleOp extends OpMode {
    // find values
    final double ARM_INTAKE = 0;
    final double ARM_OUTTAKE = 0;
    final double CLAW_HOLD = 0;
    final double CLAW_RELEASE = 0;
    final double DRONE_HOLD = 0;
    final double DRONE_RELEASE = 0.3;


    DcMotor frontLeft;
    DcMotor frontRight;
    DcMotor backLeft;
    DcMotor backRight;
    DcMotor slide;
    Servo claw;
    Servo arm;
    Servo drone;

    public void driveMecanum(double left_y, double left_x, double right_x){
        double maxPower = Math.max(Math.abs(left_y) + Math.abs(left_x) + Math.abs(right_x), 1);
        frontLeft.setPower((left_y - left_x - right_x) / maxPower);
        frontRight.setPower((left_y + left_x + right_x) / maxPower);
        backLeft.setPower((left_y + left_x - right_x) / maxPower);
        backRight.setPower((left_y - left_x + right_x) / maxPower);
    }

    public void dPadMove(String direction) {
        int pos = slide.getCurrentPosition();

        if(direction.equals("up")) {
            pos += 75;
        }
        else if(direction.equals("down") && pos > 75) {
            pos -= 75;
        }

        slide.setTargetPosition(pos);
        slide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        slide.setPower(0.75);

        if (slide.getCurrentPosition() == pos) {
            slide.setPower(0);
        }
    }

    @Override
    public void init() {
        frontLeft = hardwareMap.get(DcMotor.class, "frontLeft");
        backLeft = hardwareMap.get(DcMotor.class, "backLeft");
        frontRight = hardwareMap.get(DcMotor.class, "frontRight");
        backRight = hardwareMap.get(DcMotor.class, "backRight");
        claw = hardwareMap.get(Servo.class,"claw");
        arm = hardwareMap.get(Servo.class,"arm");
        slide = hardwareMap.get(DcMotor.class, "slide");
        drone = hardwareMap.get(Servo.class, "drone");

        slide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        slide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        arm.setDirection(Servo.Direction.REVERSE);
        claw.setDirection(Servo.Direction.REVERSE);

        frontRight.setDirection(DcMotorSimple.Direction.FORWARD);
        backRight.setDirection(DcMotorSimple.Direction.FORWARD);
        frontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeft.setDirection(DcMotorSimple.Direction.REVERSE);

        frontLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        frontRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


    }

    @Override
    public void loop() {
        driveMecanum(gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x);

        if(gamepad1.dpad_up) {
            dPadMove("up");
        }
        else if (gamepad1.dpad_down) {
            dPadMove("down");
        }

        if (gamepad1.y) {
            arm.setPosition(ARM_OUTTAKE);
        }
        else if (gamepad1.a) {
            arm.setPosition(ARM_INTAKE);
            claw.setPosition(CLAW_RELEASE);
        }
        else if (gamepad1.x) {
            claw.setPosition(CLAW_HOLD);
        }
        else if (gamepad1.b) {
            claw.setPosition(CLAW_RELEASE);
        }

        if (gamepad1.left_trigger > 0.75) {
            drone.setPosition(DRONE_RELEASE);
        } else if (gamepad1.right_trigger > 0.75) {
            drone.setPosition(DRONE_HOLD);
        }
    }
}
