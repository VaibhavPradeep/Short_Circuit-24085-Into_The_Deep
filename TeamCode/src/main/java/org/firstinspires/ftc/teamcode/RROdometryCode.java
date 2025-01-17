package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous(name = "RROdometryCode")
public class RROdometryCode extends OpMode {

    IntakeLiftCamera ILC = new IntakeLiftCamera();
    Pose2d initialPose = new Pose2d(startX, startY, Math.toRadians(startHeading));
    Pose2d nextPose = new Pose2d(startX, -43, Math.toRadians(startHeading));
    MecanumDrive drive = new MecanumDrive(hardwareMap, initialPose);

    ElapsedTime timer = new ElapsedTime();
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
    int visionOutputPosition = 1;

    int fourbarPosition = 0;

//    public void scoreFirstSpecimen() {
//        timer.reset();
//        // time
//        ILC.standbyOutake();
//        if (timer.milliseconds() > 1000) {
//            fourbarPosition = 250;
//            if (timer.milliseconds() > 2000) {
//                ILC.depositOutake();
//                if (timer.milliseconds() > 3000) {
//                    ILC.collectBrickClaw();
//                    if (timer.milliseconds() > 4000) {
//                        ILC.standbyOutake();
//                    }
//                }
//            }
//        }
//    }

    public void waitForTime(double secs){
        timer.reset();
        while (timer.time() < secs){
            ILC.DR4BMove(fourbarPosition);
        }
    }

    public void scoreFirstSpecimen(){
        ILC.standbyOutake();
        waitForTime(1);
        fourbarPosition = 250;
        waitForTime(1);
        ILC.collectBrickClaw();
        waitForTime(1);
        ILC.standbyOutake();
    }

    @Override
    public void init() {
        // Initial Pose
        MecanumDrive drive = new MecanumDrive(hardwareMap, initialPose);
        ILC.initIntakeLiftCamera(hardwareMap);

        drive.leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        drive.rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        drive.leftBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        drive.rightBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Vision here that outputs position
    }

    @Override
    public void start(){
        ILC.holdBrickClaw();
        ILC.standbyOutake();

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

    @Override
    public void loop() {
    // Trajectory setup for tab1
        ILC.DR4BMove(fourbarPosition);
    }
}
