package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

@Autonomous(name = "basket")
public class BasketSide extends OpMode {

    Drivetrain auto = new Drivetrain();

    final double speed = 0.6;

    final int driveToRack = 23;

    final double turnAroundForSpecimen = 180;


    // 0iiok

    @Override
    public void init() {
        auto.init(hardwareMap);
    }

    @Override
    public void loop() {

        // move forward to basket

        // turn around
        auto.drive(0.3,35);



        //go up a bit

    }
}
