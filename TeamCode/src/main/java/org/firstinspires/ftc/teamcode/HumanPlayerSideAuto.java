package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous(name = "HumanPlayerSideAuto")
public class HumanPlayerSideAuto extends LinearOpMode {
    IntakeLiftCamera ILC = new IntakeLiftCamera();
    Drivetrain DT = new Drivetrain(this);
    ElapsedTime timer = new ElapsedTime();

    @Override
    public void runOpMode() throws InterruptedException {
        ILC.initIntakeLiftCamera(hardwareMap);
        DT.initDrivetrain(hardwareMap);
        DT.initGyro(hardwareMap);
        ILC.collectSpecimen();

        waitForStart();

        while(opModeIsActive()) {
            DT.imu.resetYaw();
            ILC.resetDR4BMotors();

            telemetry.addData("Before holdSpecimen", "Position: " + ILC.specimenServo.getPosition());

            // scores specimen
            waitForSeconds(0.1);
            DT.drive(0.3,-31);
            waitForSeconds(0.5);
            ILC.holdSpecimen();
            waitForSeconds(0.5);
            ILC.moveDR4BMotorsAuto(2600);
            waitForSeconds(0.5);
            DT.strafe("right", 0.3, 10);
            waitForSeconds(0.5);
            DT.drive(0.3,27);
            waitForSeconds(0.5);
            DT.turn(0.3,90,"left");
            waitForSeconds(0.5);
            DT.drive(0.3, -9.6);
            waitForSeconds(0.5); // Wait for 1 second to observe the change
            ILC.moveDR4BMotorsAuto(-1000);
            waitForSeconds(0.5);
            ILC.collectSpecimen();
            waitForSeconds(0.5);
            DT.drive(0.3, 1);
            waitForSeconds(0.5);
            DT.strafe("left", 0.3, 12.5);
            waitForSeconds(0.5);
            DT.turn(0.3,5,"right");
            waitForSeconds(0.5);
            DT.drive(0.3,-5.5);
            waitForSeconds(0.5);
            DT.strafe("left", 0.3, 3);
            waitForSeconds(0.5);
            DT.turn(0.3,90,"left");
            waitForSeconds(0.5);
            DT.strafe("right",0.3,23);
            waitForSeconds(0.5);

            break;
            /*
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
            */

        }
    }

    // Custom wait function
    private void waitForSeconds(double seconds) {
        timer.reset(); // Reset the timer
        while (opModeIsActive() && timer.seconds() < seconds) {
            // Optionally, you can add a small sleep to prevent busy waiting
            sleep(50); // Sleep for 50 milliseconds
        }
    }
}