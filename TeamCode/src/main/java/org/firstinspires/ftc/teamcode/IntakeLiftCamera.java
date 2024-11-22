package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class IntakeLiftCamera {
    // Motors and Servos
    // Double Reverse Four Bar
    DcMotor leftDR4BMotor;
    DcMotor rightDR4BMotor;

    // Intake
    CRServo leftIntakeServo;
    CRServo rightIntakeServo;

    // Coaxial Virtual Four Bar
    Servo leftCV4BServo;
    Servo rightCV4BServo;
    Servo rotateIntakeServo;

    // Outake
    Servo leftOutakeServo;
    Servo rightOutakeServo;

    // Positions/other
    // Double Reverse Four Bar
    int[] maxPositions = {4200, 4200}; // potentially 4300
    int[] minPositions = {0, 0};
    int slideSpeed = 1;

    // Intake
    final double INTAKE_COLLECT    = -1.0;
    final double INTAKE_OFF        =  0.0;
    final double INTAKE_DEPOSIT    =  0.5;

    // Coaxial Virtual Four Bar
    // Virtual Four Bar
    final double LEFT_CV4B_INTAKE = 1;
    final double RIGHT_CV4B_INTAKE = 1;
    final double LEFT_CV4B_TRANSFER = 0;
    final double RIGHT_CV4B_TRANSFER = 0;

    // Rotate Intake
    final double ROTATE_FOR_INTAKE = 0;
    final double ROTATE_FOR_TRANSFER = 1.0;

    // Outake
    final double LEFT_OUTAKE_TRANSFER = 0;
    final double RIGHT_OUTAKE_TRANSFER = 0;
    final double LEFT_OUTAKE_DEPOSIT = 1.0;
    final double RIGHT_OUTAKE_DEPOSIT = 1.0;

    // Double Reverse Four Bar
    public void dPadMove(String direction) {
        int rightPos = rightDR4BMotor.getCurrentPosition();
        int leftPos = leftDR4BMotor.getCurrentPosition();

        if(direction.equals("up") && rightPos <= maxPositions[0] && leftPos <= maxPositions[1]) {
            rightPos += 100;
            leftPos += 100;
        }
        else if(direction.equals("down") && rightPos >= minPositions[0] && leftPos >= minPositions[1]) {
            rightPos -= 100;
            leftPos -= 100;
        }

        rightDR4BMotor.setTargetPosition(rightPos);
        rightDR4BMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightDR4BMotor.setPower(slideSpeed);

        leftDR4BMotor.setTargetPosition(leftPos);
        leftDR4BMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftDR4BMotor.setPower(slideSpeed);
    }

    // Intake
    public void intakeIn() {
        leftIntakeServo.setPower(INTAKE_COLLECT);
        rightIntakeServo.setPower(INTAKE_COLLECT);
    }

    public void intakeOut() {
        leftIntakeServo.setPower(INTAKE_DEPOSIT);
        rightIntakeServo.setPower(INTAKE_DEPOSIT);
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

    public void transferOrZeroCV4B() {
        leftCV4BServo.setPosition(LEFT_CV4B_TRANSFER);
        rightCV4BServo.setPosition(RIGHT_CV4B_TRANSFER);
    }

    // Coaxial Mechanism, Rotate Intake
    public void RotateToIntake() {
        rotateIntakeServo.setPosition(ROTATE_FOR_INTAKE);
    }

    public void RotateToTransfer() {
        rotateIntakeServo.setPosition(ROTATE_FOR_TRANSFER);
    }

    // Outake
    public void transferOrZeroOutake() {
        leftOutakeServo.setPosition(LEFT_OUTAKE_TRANSFER);
        rightOutakeServo.setPosition(RIGHT_OUTAKE_TRANSFER);
    }

    public void DepositOutake() {
        leftOutakeServo.setPosition(LEFT_OUTAKE_DEPOSIT);
        rightOutakeServo.setPosition(RIGHT_OUTAKE_DEPOSIT);
    }

    public void initIntakeLiftCamera(HardwareMap hwMap) {
        leftDR4BMotor = hwMap.get(DcMotor.class, "leftDR4BMotor");
        rightDR4BMotor = hwMap.get(DcMotor.class, "rightDR4BMotor");

        leftIntakeServo = hwMap.get(CRServo.class, "leftIntakeServo");
        rightIntakeServo = hwMap.get(CRServo.class, "rightIntakeServo");

        leftCV4BServo = hwMap.get(Servo.class, "leftCV4BServo");
        rightCV4BServo = hwMap.get(Servo.class, "rightCV4BServo");

        rotateIntakeServo = hwMap.get(Servo.class, "rotateIntakeServo");

        leftOutakeServo = hwMap.get(Servo.class, "leftOutakeServo");
        rightOutakeServo = hwMap.get(Servo.class, "rightOutakeServo");

        leftDR4BMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightDR4BMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        leftDR4BMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightDR4BMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        leftDR4BMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        rightDR4BMotor.setDirection(DcMotorSimple.Direction.FORWARD);
    }

}
