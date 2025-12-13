package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvInternalCamera;


@Autonomous(name = "RedBackstageAuto")

public class RedBackstageAuto extends LinearOpMode {
    IntakeLiftCamera ILC = new IntakeLiftCamera();
    Drivetrain DT = new Drivetrain(this);


    @Override public void runOpMode() {

        ILC.initILC(hardwareMap);

        DT.initDrivetrain(hardwareMap);
        DT.initGyro(hardwareMap);

        waitForStart();
        DT.initGyro(hardwareMap);

        // Loop and update the dashboard
        while (opModeIsActive()) {
            // score starting pixel
//            ILC.deployIntake();
//            ILC.closeBox();
//            ILC.armServo.setPosition(ILC.armSafe);
//            ILC.holdAuto();
//            DT.imu.resetYaw();
//
//
//            DT.drive(0.2, 29);
//
//            DT.initGyro(hardwareMap);
//
//            DT.drive(0.2, -2);
//
//            //DT.turnButBetterThanTheOldTurn(0.3, 85, "left");
//            //DT.turnToAngle(0.2, "closest", 90);
//            DT.turnButDoesntResetAngleUnlikeTurnButBetterThanTheOldTurn(0.3,85,"left");
////                DT.strafe("left", 0.2, 2);
//            DT.drive(0.2, 12);
//
//            ILC.leaveAuto();
//
//            DT.strafe("right",0.2,4);
//
//            ILC.intakeGo(-0.2);
//            DT.drive(0.2, -35);
//            ILC.intakeGo(0);
//            DT.strafe("right",0.2,5);
//
//            ILC.changeClawPos("out");
//
//
//
//            wait(1000);
//
//            DT.drive(0.2, -10);
//            ILC.openBox();
//
//            wait(1000);
//
//            DT.drive(0.2, 10);
//
//            ILC.changeClawPos("in");
//            wait(1000);
//
//            DT.drive(0.2, -20);

            break;
        }
    }

    public void wait(int milliseconds) {
        ElapsedTime waitTimer = new ElapsedTime();
        waitTimer.startTime();
        while(waitTimer.milliseconds() < milliseconds) {
            if(isStopRequested()) {
                break;
            }
        }
    }
}