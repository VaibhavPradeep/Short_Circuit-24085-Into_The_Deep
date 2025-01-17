package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@Config
@TeleOp(name = "OdometryHPSAuto")
public class OdometryHPSAuto extends LinearOpMode {


    // Configurable variables
    public static double startX = 9.5; // Starting X position
    public static int DR4B_TICKS = 0;
    public static double startY = -72; // Starting Y position
    public static double startHeading = 270; // Starting heading in degrees
    public static double lineToYSplineHeading1 = -43; // Target Y position for spline heading
    public static double lineToYSplineHeading2 = -48;
    public static double lineToXSplineHeading3 = 36;
    public static double lineToYSplineHeading4 = -4;
    public static double strafeToX5 = 48;
    public static double strafeToY5 = -4;
    public static double lineToYSplineHeading6 = -65;
    public static double turnAngle7 = 180;


    public void waitSeconds(double seconds) {
        // Get the current time in milliseconds
        long startTime = System.currentTimeMillis();
        long waitTime = (long) (seconds * 1000); // Convert seconds to milliseconds

        // Loop until the specified wait time has passed
        while (opModeIsActive() && (System.currentTimeMillis() - startTime) < waitTime) {
            // You can add any additional logic here if needed
            idle(); // Yield to other processes
        }
    }

    public void scoreFirstSpecimen() {
        ILC.standbyOutake();
        waitSeconds(1);
        ILC.DR4BMove(250);
        waitSeconds(1);
        ILC.collectBrickClaw();
        waitSeconds(1);
    }

    IntakeLiftCamera ILC = new IntakeLiftCamera();

    @Override
    public void runOpMode() throws InterruptedException {
        // Initial Pose
        Pose2d initialPose = new Pose2d(startX, startY, Math.toRadians(startHeading));
        Pose2d nextPose = new Pose2d(startX, -43, Math.toRadians(startHeading));
        MecanumDrive drive = new MecanumDrive(hardwareMap, initialPose);
        ILC.initIntakeLiftCamera(hardwareMap);

        drive.leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        drive.rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        drive.leftBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        drive.rightBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Vision here that outputs position
        int visionOutputPosition = 1;

        // Trajectory setup for tab1
        TrajectoryActionBuilder tab1 = drive.actionBuilder(initialPose)
                .setTangent(Math.toRadians(270)) // Tangent aligns with downward Y movement
                .lineToY(lineToYSplineHeading1) // Valid for downward Y movement
                .waitSeconds(1);

        // Now create tab2 for the remaining trajectory
        TrajectoryActionBuilder tab2 = drive.actionBuilder(nextPose)
                .setTangent(Math.toRadians(90)) // Tangent aligns with upward Y movement
                .lineToY(lineToYSplineHeading2) // Valid for upward Y movement
                .setTangent(Math.toRadians(0)) // Tangent aligns with rightward X movement
                .lineToX(lineToXSplineHeading3) // Valid for rightward X movement
                .waitSeconds(1)
                .setTangent(Math.toRadians(270)) // Tangent aligns with downward Y movement
                .lineToY(lineToYSplineHeading4) // Valid for downward Y movement
                .setTangent(Math.toRadians(0)) // Tangent aligns with rightward X movement
                .strafeTo(new Vector2d(strafeToX5, strafeToY5)) // Valid for simultaneous X-Y movement
                .waitSeconds(1)
                .setTangent(Math.toRadians(90)) // Tangent aligns with downward Y movement
                .lineToY(lineToYSplineHeading6)
                .turn(Math.toRadians(turnAngle7));

        Action trajectoryActionCloseOut = tab2.endTrajectory().fresh()
                .build();



        // Wait for start signal
        while (!isStopRequested() && !opModeIsActive()) {
            int position = visionOutputPosition;
            telemetry.addData("Position during Init", position);
            telemetry.update();
        }

        int startPosition = visionOutputPosition;
        telemetry.addData("Starting Position", startPosition);
        telemetry.update();

        waitForStart();

        waitSeconds(0.5);

        telemetry.addData("left dr4b position", ILC.leftDR4BMotor.getCurrentPosition());
        telemetry.addData("right dr4b position", ILC.rightDR4BMotor.getCurrentPosition());

        if (isStopRequested()) return;

        Actions.runBlocking(
                new SequentialAction(
                        tab1.build()
                )
        );

        scoreFirstSpecimen();

        Actions.runBlocking(
                new SequentialAction(
                        trajectoryActionCloseOut
                )
        );
    }
}
