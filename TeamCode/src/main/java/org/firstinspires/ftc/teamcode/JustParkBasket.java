package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous(name = "Just Park Basket")
public class JustParkBasket extends LinearOpMode {

    public static double startX = -9.5; // Starting X position
    public static double startY = -72; // Starting Y position
    public static double startHeading = 270;

    @Override
    public void runOpMode() throws InterruptedException {
        Pose2d initialPose = new Pose2d(startX, startY, Math.toRadians(startHeading));
        MecanumDrive drive = new MecanumDrive(hardwareMap, initialPose);

        TrajectoryActionBuilder tab1 = drive.actionBuilder(initialPose)
                .waitSeconds(0.5)
                .setTangent(Math.toRadians(270)) // Tangent aligns with downward Y movement
                .lineToY(-68) // Valid for downward Y movement
                .waitSeconds(0.5)
                .setTangent(180)
                .lineToX(-60);

        waitForStart();

        if (opModeIsActive()) {
            Actions.runBlocking(
                    new SequentialAction(
                            tab1.build()
                    )
            );
        }
    }
}