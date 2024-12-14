package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous(name = "HumanPlayerSideAuto")
public class HumanPlayerSideAuto extends LinearOpMode {
    IntakeLiftCamera ILC = new IntakeLiftCamera();
    Drivetrain DT = new Drivetrain(this);

    @Override
    public void runOpMode() {
        ILC.initIntakeLiftCamera(hardwareMap);
        DT.initDrivetrain(hardwareMap);
        DT.initGyro(hardwareMap);

        waitForStart();

        while(opModeIsActive()) {
            DT.imu.resetYaw();
            ILC.resetDR4BMotors();

            // scores specimen
            DT.drive(0.5,20);
            ILC.moveDR4BMotorsAuto(2800);
            DT.drive(0.5,4);
            ILC.moveDR4BMotorsAuto(-80);
            ILC.collectSpecimen();
            ILC.moveDR4BMotorsAuto(-420);

            // goes to first block and pushes it into zone
            DT.drive(0.5,-12);
            DT.strafe("right",0.5, 28);
            DT.drive(0.5, 27);
            DT.turn(0.5, 90, "left");
            DT.drive(0.5, 10);
            DT.strafe("left", 0.5, 39);

            // second block
            DT.strafe("right", 0.5, 39);

            /*
             all methods:
             drive
             strafe
             turn
             move any servos
             move DR4B whatever ticks
             */
            break;
        }

    }
}
