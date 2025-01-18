package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import androidx.annotation.NonNull;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import org.firstinspires.ftc.teamcode.MecanumDrive;

import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

@Config
@Autonomous(name = "newer auto RR")
public class NewerRRCode extends LinearOpMode {
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

    public class Claw {
        private Servo claw;

        public Claw(HardwareMap hardwareMap) {
            claw = hardwareMap.get(Servo.class, "clawServo");
        }

        public class CloseClaw implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                claw.setPosition(0);
                return false;
            }
        }
        public Action closeClaw() {
            return new Claw.CloseClaw();
        }

        public class OpenClaw implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                claw.setPosition(0.7);
                return false;
            }
        }
        public Action openClaw() {
            return new Claw.OpenClaw();
        }
    }

    public class OutakeServos {
        private Servo leftOutakeServo;
        private Servo rightOutakeServo;
        // ergjeroigjeri 8erg84gugr8egru

        public OutakeServos(HardwareMap hardwareMap) {
            leftOutakeServo = hardwareMap.get(Servo.class, "leftOutakeServo");
            rightOutakeServo = hardwareMap.get(Servo.class, "rightOutakeServo");
        }

        public class InitOutake implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                leftOutakeServo.setPosition(0.35);
                rightOutakeServo.setPosition(0.35);
                return false;
            }
        }
        public Action initOutake() {
            return new OutakeServos.InitOutake();
        }

        public class PreScoreOutake implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                leftOutakeServo.setPosition(0.7);
                rightOutakeServo.setPosition(0.7);
                return false;
            }
        }
        public Action preScoreOutake() {
            return new OutakeServos.PreScoreOutake();
        }

        public class ScoreOutake implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                leftOutakeServo.setPosition(0.8);
                rightOutakeServo.setPosition(0.8);
                return false;
            }
        }
        public Action scoreOutake() {
            return new OutakeServos.ScoreOutake();
        }
    }

    @Override
    public void runOpMode() {
        Pose2d initialPose = new Pose2d(startX, startY, Math.toRadians(startHeading));
        Pose2d nextPose = new Pose2d(startX, -43, Math.toRadians(startHeading));
        MecanumDrive drive = new MecanumDrive(hardwareMap, initialPose);
        Claw claw = new Claw(hardwareMap);
        OutakeServos outakeServos = new OutakeServos(hardwareMap);

        TrajectoryActionBuilder tab1 = drive.actionBuilder(initialPose)
                .setTangent(Math.toRadians(270)) // Tangent aligns with downward Y movement
                .lineToY(lineToYSplineHeading1) // Valid for downward Y movement
                .waitSeconds(1);
        Action tab1build = tab1.build();

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
                        claw.closeClaw(),
                        outakeServos.initOutake(),
                        tab1build,
                        outakeServos.preScoreOutake(),
                        outakeServos.scoreOutake(),
                        claw.openClaw(),
                        trajectoryActionCloseOut
                )
        );
    }
}
