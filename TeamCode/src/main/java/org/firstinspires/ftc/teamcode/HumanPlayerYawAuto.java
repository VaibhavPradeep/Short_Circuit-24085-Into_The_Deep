package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
@Autonomous( name = "Basketism")
public class HumanPlayerYawAuto extends OpMode {

    DriveTrainYawMethods auto = new DriveTrainYawMethods();
    // IntakeLiftCameraAuto ILCA = new IntakeLiftCameraAuto();

    final double speed = 0.6;

    final int driveToRack = 20;

    final double turnAroundForSpecimen = 180;

    @Override
    public void init() {
        auto.initDrivetrainYaw(hardwareMap);
        // ILCA.initILCA(hardwareMap);
    }

    @Override
    public void loop() {

        // move forward to basket
        auto.drive(speed, driveToRack);

        // turn around
        auto.strafe("right", 0.6, 23);

        //go up a bit

    }
}
