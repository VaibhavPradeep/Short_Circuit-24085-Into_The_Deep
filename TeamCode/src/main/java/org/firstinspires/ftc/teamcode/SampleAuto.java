package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@TeleOp(name = "Sample Auto")
public class SampleAuto extends LinearOpMode {

    Drivetrain dt = new Drivetrain();


    boolean isBlue = false;

    @Override
    public void runOpMode() {
        dt.initDrivetrain(hardwareMap);
        dt.initGyro(hardwareMap);


        int sideTurn = isBlue ? 90 : -90;
        int sideDrive = -12;
        int inwardDrive = 30;
        int goalTurn = isBlue ? -90 : 90;

        waitForStart();

        // Shoot now

        dt.drive(-24, 0.7);
        dt.turn(sideTurn, 0.7);
        dt.drive(sideDrive, 0.7);
        // Start intake here
        dt.drive(inwardDrive, 0.7);
        // Stop intake here
        dt.turn(goalTurn, 0.7);
        dt.drive(24, 0.7);

        // Shoot now

        dt.drive(-48, 0.7);
        dt.turn(sideTurn, 0.7);
        dt.drive(sideDrive, 0.7);
        // Start intake here
        dt.drive(inwardDrive, 0.7);
        // Stop intake here
        dt.turn(goalTurn, 0.7);
        dt.drive(48, 0.7);

        // Shoot now

        dt.drive(-72, 0.7);
        dt.turn(sideTurn, 0.7);
        dt.drive(sideDrive, 0.7);
        // Start intake here
        dt.drive(inwardDrive, 0.7);
        // Stop intake here
        dt.turn(goalTurn, 0.7);
        dt.drive(72, 0.7);

        // Shoot now
    }
}