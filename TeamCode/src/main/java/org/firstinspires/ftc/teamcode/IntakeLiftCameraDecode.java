package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.dfrobot.HuskyLens;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.internal.system.Deadline;

import java.util.concurrent.TimeUnit;

public class IntakeLiftCameraDecode {

    
    DcMotor intakeMotor;
    DcMotor pitchMotor;
    DcMotor rotationMotor;
    DcMotor shootingMotor;
    
    CRServo transferServo;
    CRServo sorterServo;

    ColorSensor colorSensor;
    
    public void initILC(HardwareMap hwMap) {
        intakeMotor = hwMap.get(DcMotor.class, "intakeMotor");
        pitchMotor = hwMap.get(DcMotor.class, "pitchMotor");
        rotationMotor = hwMap.get(DcMotor.class, "rotationMotor");
        shootingMotor = hwMap.get(DcMotor.class, "shootingMotor");
        colorSensor = hwMap.get(ColorSensor.class,"colorSensor");

        rotationMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        pitchMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rotationMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        
    }
    
    public void IntakeOn(){
        intakeMotor.setPower(1);
    }

    public void IntakeOff(){
        intakeMotor.setPower(0);
    }

    public void VerticalTurretManualMove(String direction) {
        int Pos = pitchMotor.getCurrentPosition();

        if(direction.equals("up")) {
            Pos += 100;
        }
        else if(direction.equals("down")) {Pos -= 100;
        }

        pitchMotor.setTargetPosition(Pos);
        pitchMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        pitchMotor.setPower(1);

        if (pitchMotor.getCurrentPosition() == Pos) {
            pitchMotor.setPower(0);
        }
    }

    public void LateralTurretManualMove(String direction) {
        int Pos = rotationMotor.getCurrentPosition();

        if(direction.equals("left")) {
            Pos += 100;
        }
        else if(direction.equals("right")) {Pos -= 100;
        }

        rotationMotor.setTargetPosition(Pos);
        rotationMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rotationMotor.setPower(1);

        if (rotationMotor.getCurrentPosition() == Pos) {
            rotationMotor.setPower(0);
        }
    }

    public void TransferOn() {
        transferServo.setPower(1);
    }

    public void TransferOff() {
        transferServo.setPower(0);
    }

    public void SorterOn() {
        sorterServo.setPower(1);
    }

    public void SorterOff() {
        sorterServo.setPower(0);
    }

    public void ShootingOn(){
        shootingMotor.setPower(1);
    }

    public void ShootingOff(){
        shootingMotor.setPower(0);
    }

    public boolean detectGreen() {
        if (colorSensor.green() == 150) {
            return true;
        } else {
            return false;
        }
    }

    public boolean detectPurple() {
        if (colorSensor.red() >= 140 && colorSensor.blue() >= 125) {
            return true;
        } else {
            return false;
        }
    }
}
