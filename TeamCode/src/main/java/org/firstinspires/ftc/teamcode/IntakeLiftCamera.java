package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class IntakeLiftCamera {
    // Notes: rotate intake servo should be reversed
    // left CV4B reversed, right is forward

    Servo intakeClawServo;
    // Motors and Servos
    // Double Reverse Four Bar
    DcMotor leftDR4BMotor;
    DcMotor rightDR4BMotor;

    // Intake
    CRServo leftIntakeServo;
    CRServo rightIntakeServo;
    Servo stopServo;

    // Coaxial Virtual Four Bar
    Servo leftCV4BServo;
    Servo rightCV4BServo;
    Servo rotateIntakeServo;

    // Outake
    Servo leftOutakeServo;
    Servo rightOutakeServo;
    Servo wristServo;

    // Specimen
    Servo clawServo;

    // Intake
    final double INTAKE_COLLECT    = -1.0;
    final double INTAKE_OFF        =  0.0;
    final double INTAKE_DEPOSIT    =  0.5;

    // Coaxial Virtual Four Bar
    // Virtual Four Bar
    // 0.2 cv4b, 0.4 rotate intake servo for getting in submersible
    // 0.255 for cv4b and  0.7 for rotate intake to get a thing from submersible
    // 0 for pick and 0.34 for other picking, wrist servo
    // all 0.12 for rotate intake for sagging, 0 for cv4b
    // 0 for everything not sagging


    // Outake
    final double LEFT_OUTAKE_TRANSFER = 0.35;
    final double RIGHT_OUTAKE_TRANSFER = 0.35;
    final double LEFT_OUTAKE_DEPOSIT = 0.77;
    final double RIGHT_OUTAKE_DEPOSIT = 0.77;
    final double LEFT_OUTAKE_SPECIMEN_COLLECT = 1;
    final double RIGHT_OUTAKE_SPECIMEN_COLLECT = 1;

    // Specimen
    final double BRICK_HOLD = 0;
    final double BRICK_COLLECT = 0.3;

    // Double Reverse Four Bar
    public void dPadMove(String direction) {
        int rightPos = rightDR4BMotor.getCurrentPosition();
        int leftPos = leftDR4BMotor.getCurrentPosition();

        if(direction.equals("up")) {
            rightPos += 50;
            leftPos += 50;
        }
        else if(direction.equals("down")) {
            rightPos -= 50;
            leftPos -= 50;
        }

        leftDR4BMotor.setTargetPosition(leftPos);
        rightDR4BMotor.setTargetPosition(rightPos);
        leftDR4BMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightDR4BMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftDR4BMotor.setPower(1);
        rightDR4BMotor.setPower(1);

        if (leftDR4BMotor.getCurrentPosition() == leftPos && rightDR4BMotor.getCurrentPosition() == rightPos) {
            leftDR4BMotor.setPower(0);
            rightDR4BMotor.setPower(0);
        }
    }

    public void DR4BMove(int ticks) {
        leftDR4BMotor.setTargetPosition(ticks);
        rightDR4BMotor.setTargetPosition(ticks);
        leftDR4BMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightDR4BMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftDR4BMotor.setPower(1);
        rightDR4BMotor.setPower(1);

        // Wait until the motors reach the target position
        if (leftDR4BMotor.getCurrentPosition() == ticks && rightDR4BMotor.getCurrentPosition() == ticks) {
            leftDR4BMotor.setPower(0);
            rightDR4BMotor.setPower(0);
        }

        // Stop the motors once the target position is reached
        leftDR4BMotor.setPower(0);
        rightDR4BMotor.setPower(0);
    }

    public void DR4BMove(int ticks, double kp) {
        leftDR4BMotor.setTargetPosition(ticks);
        rightDR4BMotor.setTargetPosition(ticks);
        leftDR4BMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightDR4BMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftDR4BMotor.setPower(Math.abs(ticks - leftDR4BMotor.getCurrentPosition()) * kp);
        rightDR4BMotor.setPower(1);
    }


    //Coaxial Virtual Four Bar
    // Virtual Four Bar
    public void collectCV4B() {
        leftCV4BServo.setPosition(0.255);
        rightCV4BServo.setPosition(0.255);
        rotateIntakeServo.setPosition(0.7);
    }

    public void submersibleCV4B() {
        leftCV4BServo.setPosition(0.2);
        rightCV4BServo.setPosition(0.2);
        rotateIntakeServo.setPosition(0.4);
    }

    public void zeroCV4B() {
        leftCV4BServo.setPosition(0);
        rightCV4BServo.setPosition(0);
        rotateIntakeServo.setPosition(0);
    }

    public void sagggingCV4B () {
        leftCV4BServo.setPosition(0);
        rightCV4BServo.setPosition(0);
        rotateIntakeServo.setPosition(0.12);
    }

    public void normalPickup() {
        wristServo.setPosition(0);
    }

    public void turnedPickup() {
        wristServo.setPosition(0.34);
    }


    // Outake
    public void standbyOutake() {
        leftOutakeServo.setPosition(LEFT_OUTAKE_TRANSFER);
        rightOutakeServo.setPosition(RIGHT_OUTAKE_TRANSFER);
    }

    public void depositOutake() {
        leftOutakeServo.setPosition(LEFT_OUTAKE_DEPOSIT);
        rightOutakeServo.setPosition(RIGHT_OUTAKE_DEPOSIT);
    }

    public void specimenCollectOutake() {
        leftOutakeServo.setPosition(LEFT_OUTAKE_SPECIMEN_COLLECT);
        rightOutakeServo.setPosition(RIGHT_OUTAKE_SPECIMEN_COLLECT);
    }

    // Specimen
    public void holdBrickClaw() {
        clawServo.setPosition(BRICK_HOLD);
    }

    public void collectBrickClaw() {
        clawServo.setPosition(BRICK_COLLECT);
    }

    public void holdBrickIntake() {
        intakeClawServo.setPosition(0);
    }

    public void collectBrickIntake() {
        intakeClawServo.setPosition(0.28);
    }

    // Init
    public void initIntakeLiftCamera(HardwareMap hwMap) {
        // correct
        leftDR4BMotor = hwMap.get(DcMotor.class, "leftDR4BMotor");
        rightDR4BMotor = hwMap.get(DcMotor.class, "rightDR4BMotor");
        rightDR4BMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        // correct
        leftCV4BServo = hwMap.get(Servo.class, "leftCV4BServo");
        rightCV4BServo = hwMap.get(Servo.class, "rightCV4BServo");
        rightCV4BServo.setDirection(Servo.Direction.REVERSE);

        // correct
        rotateIntakeServo = hwMap.get(Servo.class, "rotateIntakeServo");

        // correct
        leftOutakeServo = hwMap.get(Servo.class, "leftOutakeServo");
        rightOutakeServo = hwMap.get(Servo.class, "rightOutakeServo");
        rightOutakeServo.setDirection(Servo.Direction.REVERSE);

        // correct
        clawServo = hwMap.get(Servo.class, "clawServo");
        clawServo.setDirection(Servo.Direction.REVERSE);

        intakeClawServo = hwMap.get(Servo.class, "intakeClawServo");

        wristServo = hwMap.get(Servo.class, "wristServo");


        // correct
        leftDR4BMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightDR4BMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftDR4BMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightDR4BMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public void addTelemetry(Telemetry telemetry) {
        telemetry.addData("left DR4B motor position", leftDR4BMotor.getCurrentPosition());
        telemetry.addData("right DR4B motor position", rightDR4BMotor.getCurrentPosition());

        telemetry.addData("left CV4B Servo position", leftCV4BServo.getPosition());
        telemetry.addData("right CV4B Servo position", rightCV4BServo.getPosition());

        telemetry.addData("rotate intake Servo position", rotateIntakeServo.getPosition());

        telemetry.addData("left outake Servo position", leftOutakeServo.getPosition());
        telemetry.addData("right outake Servo position", rightOutakeServo.getPosition());


        telemetry.update();
    }

}
