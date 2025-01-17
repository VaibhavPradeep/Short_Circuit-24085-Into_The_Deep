package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Config
@Autonomous(name = "RRSampleTrajectory", group = "Autonomous")
public class RRSampleTrajectory extends LinearOpMode {

    // Configurable variables for trajectories
    public static double startX = 12;
    public static double startY = -72;
    public static double startHeading = 270;
    public static double lineToYSplineHeading1 = 33;
    public static double lineToYSplineHeading2 = 48;
    public static double lineToX1 = 32;
    public static double lineToX2 = 47.5;
    public static double strafeX = 44.5;
    public static double strafeY = 30;
    public static double lineToY1 = 37;
    public static double lineToX3 = 18;
    public static double lineToXSplineHeading = 46;
    public static double turnAngle = Math.toRadians(180);

    @Override
    public void runOpMode() {
        Pose2d initialPose = new Pose2d(startX, startY, Math.toRadians(startHeading));
        MecanumDrive drive = new MecanumDrive(hardwareMap, initialPose);

        // Vision here that outputs position
        int visionOutputPosition = 1;

        TrajectoryActionBuilder tab1 = drive.actionBuilder(initialPose)
                .lineToYSplineHeading(lineToYSplineHeading1, Math.toRadians(0))
                .waitSeconds(2)
                .setTangent(Math.toRadians(270))
                .lineToY(lineToYSplineHeading2)
                .setTangent(Math.toRadians(0))
                .lineToX(lineToX1)
                .strafeTo(new Vector2d(strafeX, strafeY))
                .turn(turnAngle)
                .lineToX(lineToX2)
                .waitSeconds(3);

        TrajectoryActionBuilder tab2 = drive.actionBuilder(initialPose)
                .lineToY(lineToY1)
                .setTangent(Math.toRadians(0))
                .lineToX(lineToX3)
                .waitSeconds(3)
                .setTangent(Math.toRadians(0))
                .lineToXSplineHeading(lineToXSplineHeading, Math.toRadians(180))
                .waitSeconds(3);

        TrajectoryActionBuilder tab3 = drive.actionBuilder(initialPose)
                .lineToYSplineHeading(lineToYSplineHeading1, Math.toRadians(180))
                .waitSeconds(2)
                .strafeTo(new Vector2d(strafeX, strafeY))
                .waitSeconds(3);

        Action trajectoryActionCloseOut = tab1.endTrajectory().fresh()
                .strafeTo(new Vector2d(48, 12))
                .build();

        while (!isStopRequested() && !opModeIsActive()) {
            int position = visionOutputPosition;
            telemetry.addData("Position during Init", position);
            telemetry.update();
        }

        int startPosition = visionOutputPosition;
        telemetry.addData("Starting Position", startPosition);
        telemetry.update();
        waitForStart();

        if (isStopRequested()) return;

        Action trajectoryActionChosen;
        if (startPosition == 1) {
            trajectoryActionChosen = tab1.build();
        } else if (startPosition == 2) {
            trajectoryActionChosen = tab2.build();
        } else {
            trajectoryActionChosen = tab3.build();
        }

        Actions.runBlocking(
                new SequentialAction(
                        trajectoryActionChosen,
                        trajectoryActionCloseOut
                )
        );
    }
}
