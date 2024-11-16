package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
@Autonomous( name = "human")
public class HumanPlayerYawAuto extends OpMode {

    DriveTrainYawMethods auto = new DriveTrainYawMethods();
    // IntakeLiftCameraAuto ILCA = new IntakeLiftCameraAuto();

    final double speed = 0.3;

    final int driveToRack = 5;

    final double turnAroundForSpecimen = 180;

    @Override
    public void init() {
        auto.initDrivetrainYaw(hardwareMap);
        // ILCA.initILCA(hardwareMap);rg
    }

    @Override
    public void start() {

        // move forward to basket
        auto.drive(speed, driveToRack);

        // turn around
        auto.strafe("right", 0.3, 50);

        //go up a bit

    }

    @Override
    public void loop() {

    }
}
