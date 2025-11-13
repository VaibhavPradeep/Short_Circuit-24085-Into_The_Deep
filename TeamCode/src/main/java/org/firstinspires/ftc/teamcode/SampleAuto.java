package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name = "sample auto")
public class SampleAuto extends OpMode {


    // import classes for functions
    Drivetrain dt = new Drivetrain();

    @Override
    public void init() {
        dt.initDrivetrain(hardwareMap);
        dt.initGyro(hardwareMap);
        // init functions
    }

    @Override
    public void start() {
        // shooting first 3 balls
        dt.drive(30,0.7);
        // ex: turn here 30 degrees
    }

    @Override
    public void loop() {

    }
}
