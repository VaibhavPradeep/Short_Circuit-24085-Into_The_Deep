package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name = "RobotCentricDrive")
public class RobotCentricDrive extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {

        MecanumDriveCircuit mecanumDrive = new MecanumDriveCircuit();
        mecanumDrive.initDrive(hardwareMap);

        waitForStart();

        while (opModeIsActive()) {
            mecanumDrive.driveMecanum(gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x);
        }
    }
}
