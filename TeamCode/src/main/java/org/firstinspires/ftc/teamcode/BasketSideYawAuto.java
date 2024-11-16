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


    // 0iiok

    @Override
    public void init() {
        auto.initDrivetrainYaw(hardwareMap);
        ILCA.initILCA(hardwareMap);
    }

    @Override
    public void loop() {

        // move forward to basket

        // turn around
        auto.drive(0.3,38);

        ILCA.DR4BAndIntakeForBasket(10000, 4000);

        ILCA.intakeOff();

        ILCA.justDR4B(1000);

        ILCA.justDR4B(1000);

        ILCA.justDR4B(1000);

        ILCA.justDR4B(1000);

        ILCA.justDR4B(1000);

        ILCA.justDR4B(1000);



        //go up a bit

    }
}
