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
    Servo armOutakeServo;

    // Specimen
    Servo clawServo;

    // Intake
    final double INTAKE_COLLECT    = -1.0;
    final double INTAKE_OFF        =  0.0;
    final double INTAKE_DEPOSIT    =  0.5;

    // Coaxial Virtual Four Bar
    // Virtual Four Bar
    final double LEFT_CV4B_INTAKE = 0.22;
    final double RIGHT_CV4B_INTAKE = 0.22;

    final double LEFT_CV4B_BOX = 0.14;
    final double RIGHT_CV4B_BOX = 0.14;
    final double LEFT_CV4B_TRANSFER = 0;
    final double RIGHT_CV4B_TRANSFER  = 0;

    // Rotate Intake
    final double ROTATE_FOR_INTAKE = 0;
    final double ROTATE_FOR_TRANSFER = 0.3;

    // Outake
    final double LEFT_OUTAKE_TRANSFER = 0;
    final double RIGHT_OUTAKE_TRANSFER = 0;
    final double LEFT_OUTAKE_DEPOSIT = 0.45;
    final double RIGHT_OUTAKE_DEPOSIT = 0.45;
    final double LEFT_OUTAKE_SPECIMEN_COLLECT = 0.8;
    final double RIGHT_OUTAKE_SPECIMEN_COLLECT = 0.8;

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

        // Intake
    public void intakeIn() {
        leftIntakeServo.setPower(INTAKE_COLLECT);
        // rightIntakeServo.setPower(INTAKE_COLLECT);
    }

    public void intakeOut() {
        leftIntakeServo.setPower(INTAKE_DEPOSIT);
        // rightIntakeServo.setPower(INTAKE_DEPOSIT);
    }

    public void intakeOff() {
        leftIntakeServo.setPower(INTAKE_OFF);
        leftIntakeServo.setPower(INTAKE_OFF);
    }

    //Coaxial Virtual Four Bar
    // Virtual Four Bar
    public void collectCV4B() {
        leftCV4BServo.setPosition(LEFT_CV4B_INTAKE);
        rightCV4BServo.setPosition(RIGHT_CV4B_INTAKE);
    }

    public void submersibleCV4B() {
        leftCV4BServo.setPosition(LEFT_CV4B_BOX);
        rightCV4BServo.setPosition(RIGHT_CV4B_BOX);
    }

    public void transferOrZeroCV4B() {
        leftCV4BServo.setPosition(LEFT_CV4B_TRANSFER);
        rightCV4BServo.setPosition(RIGHT_CV4B_TRANSFER);
    }

    // Coaxial Mechanism, Rotate Intake
    public void rotateToIntake() {
        rotateIntakeServo.setPosition(ROTATE_FOR_INTAKE);
    }

    public void rotateToTransfer() {
        rotateIntakeServo.setPosition(ROTATE_FOR_TRANSFER);
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
        intakeClawServo.setPosition(1);
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
        intakeClawServo.setDirection(Servo.Direction.REVERSE);


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

        telemetry.addData("claw servo position", clawServo.getPosition());

        telemetry.update();
    }

}
