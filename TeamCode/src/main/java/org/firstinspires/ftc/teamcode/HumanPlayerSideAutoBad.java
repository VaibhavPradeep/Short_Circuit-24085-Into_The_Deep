package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous(name = "ParkHumanPlayerSideAuto")
public class HumanPlayerSideAutoBad extends LinearOpMode {
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
            DT.strafe("left",0.3,24);
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