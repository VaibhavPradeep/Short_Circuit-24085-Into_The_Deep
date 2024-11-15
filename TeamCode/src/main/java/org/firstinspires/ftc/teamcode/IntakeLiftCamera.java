
package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class IntakeLiftCamera {
    // dr4b
    DcMotor leftDR4BMotor;
    DcMotor rightDR4BMotor;

    // intake
    CRServo intakeServo;
    Servo leftArmServo;
    Servo rightArmServo;

    //Specimen
    Servo specimenServo;

    //distance sensor
    DistanceSensor distanceSensor;

    // INTAKE SERVO
    final double INTAKE_COLLECT    = -1.0;
    final double INTAKE_OFF        =  0.0;
    final double INTAKE_DEPOSIT    =  0.5;

    // CHANGE, ARM SERVO
    final double ARM_COLLECT = 0;
    final double ARM_DEPOSIT = 1;

    // CHANGE LATER yyj
    final double SPECIMEN_COLLECT = 1.0;
    final double SPECIMEN_HOLD = 0;

    double DR4B_SPEED = 0.6;

    public void initILC(HardwareMap hwMap) {
        distanceSensor = hwMap.get(DistanceSensor.class, "distanceSensor");

        leftDR4BMotor = hwMap.get(DcMotor.class, "leftDR4BMotor");
        rightDR4BMotor = hwMap.get(DcMotor.class, "rightDR4BMotor");
        leftDR4BMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightDR4BMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        
        leftDR4BMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        intakeServo = hwMap.get(CRServo.class, "intakeServo");

        specimenServo = hwMap.get(Servo.class,"specimenServo");

        distanceSensor = hwMap.get(DistanceSensor.class, "distanceSensor");

        leftArmServo = hwMap.get(Servo.class, "leftArmServo");
        rightArmServo = hwMap.get(Servo.class, "rightArmServo");

        rightArmServo.setDirection(Servo.Direction.REVERSE);

        leftArmServo.setPosition(0);
        rightArmServo.setPosition(0);
    }

    public void moveIntakeServo (boolean a1,boolean x1, boolean b1) {
        if (a1) {
            intakeServo.setPower(INTAKE_DEPOSIT);
        }
        else if (x1) {
            intakeServo.setPower(INTAKE_COLLECT);
        }
        else if (b1) {
            intakeServo.setPower(INTAKE_OFF);
        }
    }

    /* for dpad movement way
    public void moveDR4BMotors (boolean dpadup2, boolean dpaddown2) {
        int leftDR4BMotorPos = leftDR4BMotor.getCurrentPosition();
        int rightDR4BMotorPos = rightDR4BMotor.getCurrentPosition();

        if(dpadup2) {
            leftDR4BMotorPos += 50;
            rightDR4BMotorPos += 50;
        }
        else if(dpaddown2) {
            leftDR4BMotorPos -= 50;
            rightDR4BMotorPos -= 50;
        }

        leftDR4BMotor.setTargetPosition(leftDR4BMotorPos);
        rightDR4BMotor.setTargetPosition(rightDR4BMotorPos);
        leftDR4BMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightDR4BMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftDR4BMotor.setPower(DR4B_SPEED);
        rightDR4BMotor.setPower(DR4B_SPEED);

        if (leftDR4BMotor.getCurrentPosition() == leftDR4BMotorPos && rightDR4BMotor.getCurrentPosition() == rightDR4BMotorPos) {
            leftDR4BMotor.setPower(0);
            rightDR4BMotor.setPower(0);
        }

    }
     */

    public void moveDR4BMotors (double lefttrigger2, double righttrigger2) {
        int leftDR4BMotorPos = leftDR4BMotor.getCurrentPosition();
        int rightDR4BMotorPos = rightDR4BMotor.getCurrentPosition();

        double DR4BSpeed = 0;

        if(lefttrigger2 > 0.1) {
            leftDR4BMotorPos += 50;
            rightDR4BMotorPos += 50;
            DR4BSpeed = lefttrigger2;
        }
        else if(righttrigger2 > 0.1) {
            leftDR4BMotorPos -= 50;
            rightDR4BMotorPos -= 50;
            DR4BSpeed = righttrigger2;
        }

        if (DR4BSpeed > 0.65) {
            DR4BSpeed = 0.65;
        }

        leftDR4BMotor.setTargetPosition(leftDR4BMotorPos);
        rightDR4BMotor.setTargetPosition(rightDR4BMotorPos);
        leftDR4BMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightDR4BMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftDR4BMotor.setPower(DR4BSpeed);
        rightDR4BMotor.setPower(DR4BSpeed);

        if (leftDR4BMotor.getCurrentPosition() == leftDR4BMotorPos && rightDR4BMotor.getCurrentPosition() == rightDR4BMotorPos) {
            leftDR4BMotor.setPower(0);
            rightDR4BMotor.setPower(0);
        }

        DR4BSpeed = 0;
    }

    public void moveSpecimenServo(boolean leftbump1, boolean rightbump1) {
        if (leftbump1) {
            specimenServo.setPosition(SPECIMEN_COLLECT);
        }
        else if (rightbump1) {
            specimenServo.setPosition(SPECIMEN_HOLD);
        }

    }

    public void moveArmServos(boolean leftbump2, boolean rightbump2) {
        double leftservoPosition = leftArmServo.getPosition();
        double rightservoPosition = rightArmServo.getPosition();

        if (leftbump2 && !rightbump2) {
            leftservoPosition = Math.min(1.0, leftservoPosition + 0.05);
            rightservoPosition = Math.min(1.0, rightservoPosition + 0.05);
        } else if (rightbump2 && !leftbump2) {
            leftservoPosition = Math.max(0.0, leftservoPosition - 0.05);
            rightservoPosition = Math.max(0.0, rightservoPosition - 0.05);
        }

        leftArmServo.setPosition(leftservoPosition);
        rightArmServo.setPosition(rightservoPosition);

    }

    public double getDistance(DistanceUnit du){
        return distanceSensor.getDistance(du);
    }
    public void addTelemetry(Telemetry telemetry) {
        telemetry.addData("left DR4B motor position", leftDR4BMotor.getCurrentPosition());
        telemetry.addData("right DR4B motor position", rightDR4BMotor.getCurrentPosition());

        telemetry.addData("left servo pos", leftArmServo.getPosition());
        telemetry.addData("right servo pos", rightArmServo.getPosition());

        telemetry.addData("specimen servo position", specimenServo.getPosition());

        telemetry.addData("distance away from anything", getDistance(DistanceUnit.INCH));

        if (getDistance(DistanceUnit.INCH) <= 3) {
            telemetry.addLine("TOO CLOSE");
        } else if (getDistance(DistanceUnit.INCH) <= 6) {
            telemetry.addLine("GETTING CLOSE");
        }

        telemetry.update();
    }
}
