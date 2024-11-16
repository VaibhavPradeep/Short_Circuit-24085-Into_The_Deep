package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;

public class BasketSideYawAuto extends OpMode {

    DriveTrainYawMethods auto = new DriveTrainYawMethods();
    IntakeLiftCameraAuto ILCA = new IntakeLiftCameraAuto();

    final double speed = 0.6;

    final int driveToRack = 23;

    final double turnAroundForSpecimen = 180;

    @Override
    public void init() {
        auto.initDrivetrainYaw(hardwareMap);
        ILCA.initILCA(hardwareMap);
    }

    @Override
    public void loop() {

        // move forward to basket
        auto.drive(speed, driveToRack);

        // turn around
        auto.turn(speed, turnAroundForSpecimen, "left");

        //go up a bit

    }
}
