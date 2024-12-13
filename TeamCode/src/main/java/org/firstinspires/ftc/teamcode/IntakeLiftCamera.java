package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;

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

    // Specimen
    Servo specimenServo;

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
    final double LEFT_OUTAKE_DEPOSIT = 0.45;
    final double RIGHT_OUTAKE_DEPOSIT = 0.45;

    // Specimen
    final double SPECIMEN_HOLD = 0;
    final double SPECIMEN_COLLECT = 0.35;

    // Double Reverse Four Bar
    public void resetDR4BMotors () {
        leftDR4BMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightDR4BMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftDR4BMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightDR4BMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public void moveDR4BMotorsAuto (int ticks) {
        int leftDR4BMotorPos = leftDR4BMotor.getCurrentPosition();
        int rightDR4BMotorPos = rightDR4BMotor.getCurrentPosition();

        double DR4BSpeed = 0.8;

        leftDR4BMotorPos += ticks;
        rightDR4BMotorPos += ticks;

        leftDR4BMotor.setTargetPosition(leftDR4BMotorPos);
        rightDR4BMotor.setTargetPosition(rightDR4BMotorPos);
        leftDR4BMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightDR4BMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftDR4BMotor.setPower(DR4BSpeed);
        rightDR4BMotor.setPower(DR4BSpeed);

        while (leftDR4BMotor.isBusy() && rightDR4BMotor.isBusy()) {
            // Do nothing, just wait for the motors to finish moving
        }

        leftDR4BMotor.setPower(0);
        rightDR4BMotor.setPower(0);
        DR4BSpeed = 0;
    }
    public void DR4BMove(String direction, double lefttrigger2, double righttrigger2) {
        int rightPos = rightDR4BMotor.getCurrentPosition();
        int leftPos = leftDR4BMotor.getCurrentPosition();

        double DR4BSpeed = 0;

        if(lefttrigger2 > 0.1 && direction.equals("up") ) {
            rightPos += 100;
            leftPos += 100;
            DR4BSpeed = lefttrigger2;
        }
        else if(righttrigger2 > 0.1 && direction.equals("down")) {
            rightPos -= 100;
            leftPos -= 100;
            DR4BSpeed = righttrigger2;
        }

        leftDR4BMotor.setTargetPosition(leftPos);
        rightDR4BMotor.setTargetPosition(rightPos);
        leftDR4BMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightDR4BMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftDR4BMotor.setPower(DR4BSpeed);
        rightDR4BMotor.setPower(DR4BSpeed);

        while (leftDR4BMotor.isBusy() && rightDR4BMotor.isBusy()) {
            // Do nothing, just wait for the motors to finish moving
        }

        leftDR4BMotor.setPower(0);
        rightDR4BMotor.setPower(0);
        DR4BSpeed = 0;
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
    public void rotateToIntake() {
        rotateIntakeServo.setPosition(ROTATE_FOR_INTAKE);
    }

    public void rotateToTransfer() {
        rotateIntakeServo.setPosition(ROTATE_FOR_TRANSFER);
    }

    // Outake
    public void transferOrZeroOutake() {
        leftOutakeServo.setPosition(LEFT_OUTAKE_TRANSFER);
        rightOutakeServo.setPosition(RIGHT_OUTAKE_TRANSFER);
    }

    public void depositOutake() {
        leftOutakeServo.setPosition(LEFT_OUTAKE_DEPOSIT);
        rightOutakeServo.setPosition(RIGHT_OUTAKE_DEPOSIT);
    }

    // Specimen
    public void holdSpecimen() {
        specimenServo.setPosition(SPECIMEN_HOLD);
    }

    public void collectSpecimen() {
        specimenServo.setPosition(SPECIMEN_COLLECT);
    }

    // Other
    public void zeroServos() {
        leftCV4BServo.setPosition(0);
        rightCV4BServo.setPosition(0);

        rotateIntakeServo.setPosition(0);

        leftOutakeServo.setPosition(0);
        rightOutakeServo.setPosition(0);

        specimenServo.setPosition(0);
    }

    // Init
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

        specimenServo = hwMap.get(Servo.class, "specimenServo");

        leftDR4BMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightDR4BMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        leftDR4BMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightDR4BMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        leftDR4BMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        rightDR4BMotor.setDirection(DcMotorSimple.Direction.FORWARD);
    }

    public void addTelemetry(Telemetry telemetry) {
        telemetry.addData("left DR4B motor position", leftDR4BMotor.getCurrentPosition());
        telemetry.addData("right DR4B motor position", rightDR4BMotor.getCurrentPosition());

        telemetry.addData("left CV4B Servo position", leftCV4BServo.getPosition());
        telemetry.addData("right CV4B Servo position", rightCV4BServo.getPosition());

        telemetry.addData("rotate intake Servo position", rotateIntakeServo.getPosition());

        telemetry.addData("left outake Servo position", leftOutakeServo.getPosition());
        telemetry.addData("right outake Servo position", rightOutakeServo.getPosition());

        telemetry.addData("specimen servo position", specimenServo.getPosition());

        telemetry.update();
    }

}
