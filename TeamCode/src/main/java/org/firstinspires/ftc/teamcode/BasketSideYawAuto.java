package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

@Autonomous(name = "basket")
public class BasketSideYawAuto extends OpMode {

    DriveTrainYawMethods auto = new DriveTrainYawMethods();
    IntakeLiftCameraAuto ILCA = new IntakeLiftCameraAuto();

    final double speed = 0.6;

    final int driveToRack = 23;

    final double turnAroundForSpecimen = 180;
    // 0ii

    @Override
    public void init() {
        auto.initDrivetrainYaw(hardwareMap);
        ILCA.initILCA(hardwareMap);
    }

    @Override
    public void loop() {

        // move forward to basket
        auto.strafe("left", 0.3, 37);

        // turn around
        auto.drive(0.3,30);

        //go up a bit

    }
}
