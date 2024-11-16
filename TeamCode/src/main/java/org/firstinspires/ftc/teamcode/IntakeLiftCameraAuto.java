
package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

public class IntakeLiftCameraAuto {
    ElapsedTime timer = new ElapsedTime();
    // dr4b
    DcMotor leftDR4BMotor;
    DcMotor rightDR4BMotor;

    // intake
    CRServo intakeServo;
    Servo wristServo;
    Servo leftArmServo;
    Servo rightArmServo;


    //Specimen
    Servo specimenServo;

    //distance sensor
    // DistanceSensor distanceSensor;

    // DR4B
    final int[] DR4B_INTAKE = {0,0};
    final int[] DR4B_DEPOSIT = {50,50};

    final int[] DR4B_HIGH_RUNG = {100,100};

    // INTAKE SERVO
    final double INTAKE_COLLECT    = -1.0;
    final double INTAKE_OFF        =  0.0;
    final double INTAKE_DEPOSIT    =  0.5;

    // CHANGE, ARM SERVO
    final double ARM_COLLECT = 0;
    final double ARM_DEPOSIT = 1;

    // CHANGE LATER
    final double SPECIMEN_COLLECT = 1.0;
    final double SPECIMEN_HOLD = 0;

    // WRIST SERVO POSITIONS, CHANGE
    final double WRIST_SPECIMEN = 0;
    final double WRIST_INTAKE = 0.5;

    double DR4B_SPEED = 0.6;

    public void initILCA(HardwareMap hwMap) {
        //distanceSensor = hwMap.get(DistanceSensor.class, "distanceSensor");

        leftDR4BMotor = hwMap.get(DcMotor.class, "leftDR4BMotor");
        rightDR4BMotor = hwMap.get(DcMotor.class, "rightDR4BMotor");
        leftDR4BMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightDR4BMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        intakeServo = hwMap.get(CRServo.class, "intakeServo");

        specimenServo = hwMap.get(Servo.class,"specimenServo");

        //distanceSensor = hwMap.get(DistanceSensor.class, "distanceSensor");
        leftArmServo = hwMap.get(Servo.class, "leftArmServo");
        rightArmServo = hwMap.get(Servo.class, "rightArmServo");

    }

    public void DR4BAndIntakeForBasket(int milliseconds, int othermilliseconds) {
        timer.reset();
        leftDR4BMotor.setPower(1);
        rightDR4BMotor.setPower(1);
        if (timer.milliseconds() == milliseconds) {
            leftDR4BMotor.setPower(0);
            rightDR4BMotor.setPower(0);
        }
        if (timer.milliseconds() == othermilliseconds) {
            intakeServo.setPower(INTAKE_COLLECT);

        }
    }

    public void intakeOff() {
        intakeServo.setPower(INTAKE_OFF);
    }

    public void justDR4B(int milliseconds) {
        timer.reset();
        leftDR4BMotor.setPower(1);
        rightDR4BMotor.setPower(1);
        if (timer.milliseconds() == milliseconds) {
            leftDR4BMotor.setPower(0);
            rightDR4BMotor.setPower(0);
        }
    }

}
