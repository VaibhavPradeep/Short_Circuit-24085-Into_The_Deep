package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.dfrobot.HuskyLens;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

import java.util.ArrayList;

public class Sample extends OpMode {
    DcMotor motor1;
    Servo servo1;

    DistanceSensor distanceSensor;

    ColorSensor colorSensor;

    HuskyLens huskyLens;

    Limelight3A limelight3A;
    // 9 set pipelines, 1 pipeline 


    @Override
    public void init() {
        motor1 = hardwareMap.get(DcMotor.class, "motor1");
        servo1 = hardwareMap.get(Servo.class, "servo1");
        distanceSensor = hardwareMap.get(DistanceSensor.class, "distanceSensor");
        colorSensor = hardwareMap.get(ColorSensor.class, "colorSensor");

    }

    @Override
    public void loop() {
        motor1.setPower(0);
        motor1.setDirection(DcMotorSimple.Direction.REVERSE);
        motor1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        //Encoders
        motor1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motor1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor1.setTargetPosition(-500);
        motor1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motor1.getMode();

        servo1.setPosition(0);
        servo1.setDirection(Servo.Direction.REVERSE);
        double servoPosition = servo1.getPosition();

        double distance = distanceSensor.getDistance(DistanceUnit.CM);
        // (red, green, blue)
        // takes a intensity measure scales for 0-255

        int greenValue = colorSensor.green();
        if (greenValue > 175) {
            // start the intake and go forward
        }

    }
}
