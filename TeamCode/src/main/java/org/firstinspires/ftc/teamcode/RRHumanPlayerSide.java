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

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

@Config
@Autonomous(name = "RR Human Player Side")
public class RRHumanPlayerSide extends LinearOpMode {
    // Configurable variables
    public static double startX = 9.5; // Starting X position
    public static double startY = -72; // Starting Y position
    public static double startHeading = 270; // Starting heading in degrees
    public static double lineToYSplineHeading1 = -41; // Target Y position for spline heading
    public static double lineToYSplineHeading2 = -51;
    public static double lineToXSplineHeading3 = 36;
    public static double lineToYSplineHeading4 = -20;
    public static double strafeToX5 = 48;
    public static double strafeToY5 = -20;
    public static double lineToYSplineHeading6 = -69.75;
    public static double turnAngle7 = 180;
    public static int fourBarPos = 0;

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
                claw.setPosition(0.3);
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

        public OutakeServos(HardwareMap hardwareMap) {
            leftOutakeServo = hardwareMap.get(Servo.class, "leftOutakeServo");
            rightOutakeServo = hardwareMap.get(Servo.class, "rightOutakeServo");
            rightOutakeServo.setDirection(Servo.Direction.REVERSE);
        }

        public class InitOutake implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                leftOutakeServo.setPosition(0.9);
                rightOutakeServo.setPosition(0.9);
                return false;
            }
        }
        public Action initOutake() {
            return new OutakeServos.InitOutake();
        }

        public class GetSpecimenOutake implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                leftOutakeServo.setPosition(1);
                rightOutakeServo.setPosition(1);
                return false;
            }
        }
        public Action getSpecimenOutake() {
            return new GetSpecimenOutake();
        }

        public class ScoreOutake implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                leftOutakeServo.setPosition(0.77);
                rightOutakeServo.setPosition(0.77);
                return false;
            }
        }
        public Action scoreOutake() {
            return new OutakeServos.ScoreOutake();
        }
    }

    public class DR4BMove {

        private DcMotor leftDR4BMotor;
        private DcMotor rightDR4BMotor;
        public DR4BMove(HardwareMap hardwareMap) {
            leftDR4BMotor = hardwareMap.get(DcMotor.class, "leftDR4BMotor");
            rightDR4BMotor = hardwareMap.get(DcMotor.class, "rightDR4BMotor");
            rightDR4BMotor.setDirection(DcMotorSimple.Direction.REVERSE);
            leftDR4BMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            leftDR4BMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            leftDR4BMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            rightDR4BMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }

        public void DR4B(int ticks) {
            leftDR4BMotor.setTargetPosition(ticks);
            rightDR4BMotor.setTargetPosition(ticks);
            leftDR4BMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rightDR4BMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            leftDR4BMotor.setPower(1);
            rightDR4BMotor.setPower(1);

            /*
            // Wait until the motors reach the target position
            if (leftDR4BMotor.getCurrentPosition() == ticks && rightDR4BMotor.getCurrentPosition() == ticks) {
                leftDR4BMotor.setPower(0);
                rightDR4BMotor.setPower(0);
            }

            // Stop the motors once the target position is reached
            leftDR4BMotor.setPower(0);
            rightDR4BMotor.setPower(0);

             */
        }
        public class LiftUp implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                fourBarPos = 1550;
                DR4B(fourBarPos);
                return false;
            }
        }
        public Action liftUp() {
            return new DR4BMove.LiftUp();
        }
        public class LiftDown implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                fourBarPos = -600;
                DR4B(fourBarPos);
                return false;
            }
        }
        public Action liftDown() {
            return new DR4BMove.LiftDown();
        }
    }

    public class EverythingElse {
        private Servo leftCV4BServo;
        private Servo rightCV4BServo;
        private Servo rotateIntakeServo;
        private Servo intakeClawServo;
        public EverythingElse(HardwareMap hardwareMap) {
            leftCV4BServo = hardwareMap.get(Servo.class, "leftCV4BServo");
            rightCV4BServo = hardwareMap.get(Servo.class, "rightCV4BServo");
            rightCV4BServo.setDirection(Servo.Direction.REVERSE);
            rotateIntakeServo = hardwareMap.get(Servo.class, "rotateIntakeServo");
            intakeClawServo = hardwareMap.get(Servo.class, "intakeClawServo");
        }

        public class InitEverythingElse implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                leftCV4BServo.setPosition(0.2);
                rightCV4BServo.setPosition(0.2);
                rotateIntakeServo.setPosition(0);
                return false;
            }
        }
        public Action initEverythingElse() {
            return new EverythingElse.InitEverythingElse();
        }

    }

    @Override
    public void runOpMode() {
        Pose2d initialPose = new Pose2d(startX, startY, Math.toRadians(startHeading));
        Pose2d pose2 = new Pose2d(startX, -41, Math.toRadians(startHeading));
        Pose2d pose3 = new Pose2d(9.5, -51, Math.toRadians(startHeading));
        Pose2d pose4 = new Pose2d(48,-69.75, Math.toRadians(90));
        Pose2d pose5 = new Pose2d(10.5, -63, Math.toRadians(270));
        Pose2d pose6 = new Pose2d(startX, -41, Math.toRadians(startHeading));
        Pose2d pose7 = new Pose2d(10.5, -51, Math.toRadians(startHeading));
        Pose2d pose8 = new Pose2d(48,-69.75, Math.toRadians(90));
        Pose2d pose9 = new Pose2d(11.5, -63, Math.toRadians(270));
        Pose2d pose10 = new Pose2d(11.5, -51, Math.toRadians(startHeading));

        MecanumDrive drive = new MecanumDrive(hardwareMap, initialPose);
        Claw claw = new Claw(hardwareMap);
        OutakeServos outakeServos = new OutakeServos(hardwareMap);
        DR4BMove dr4BMove = new DR4BMove(hardwareMap);
        EverythingElse everythingElse = new EverythingElse(hardwareMap);

        TrajectoryActionBuilder tab1 = drive.actionBuilder(initialPose)
                .waitSeconds(3)
                .setTangent(Math.toRadians(270)) // Tangent aligns with downward Y movement
                .lineToY(lineToYSplineHeading1) // Valid for downward Y movement
                .waitSeconds(4);

        // Now create tab2 for the remaining trajectories
        TrajectoryActionBuilder tab2 = drive.actionBuilder(pose2)
                .waitSeconds(1)
                .setTangent(Math.toRadians(90)) // Tangent aligns with upward Y movement
                .lineToY(lineToYSplineHeading2); // Valid for upward Y movement

        TrajectoryActionBuilder tab3 = drive.actionBuilder(pose3)
                .waitSeconds(1)
                .setTangent(Math.toRadians(0)) // Tangent aligns with rightward X movement
                .lineToX(lineToXSplineHeading3)// Valid for rightward X movement
                .waitSeconds(0.25)
                .setTangent(Math.toRadians(270))
                .lineToY(lineToYSplineHeading4)// Valid for downward Y movement
                .waitSeconds(0.25)
                .turn(Math.toRadians(180))
                .waitSeconds(0.25)
                .setTangent(Math.toRadians(0)) // Tangent aligns with rightward X movement
                .strafeTo(new Vector2d(strafeToX5, strafeToY5)) // Valid for simultaneous X-Y movement
                .waitSeconds(0.25)
                .setTangent(Math.toRadians(270)) // Tangent aligns with downward Y movement
                .lineToY(lineToYSplineHeading6)
                .waitSeconds(1);


        TrajectoryActionBuilder tab4 = drive.actionBuilder(pose4)
                .waitSeconds(1)
                .setTangent(Math.toRadians(90))
                .lineToY(-63)
                .waitSeconds(0.25)
                .setTangent(Math.toRadians(180))
                .lineToX(10.5)
                .waitSeconds(0.25)
                .turn(Math.toRadians(180));

        TrajectoryActionBuilder tab5 = drive.actionBuilder(pose5)
                .waitSeconds(1)
                .setTangent(Math.toRadians(270))
                .lineToY(-41);

        TrajectoryActionBuilder tab6 = drive.actionBuilder(pose6)
                .waitSeconds(1)
                .setTangent(Math.toRadians(90)) // Tangent aligns with upward Y movement
                .lineToY(lineToYSplineHeading2); // Valid for upward Y movement

        TrajectoryActionBuilder tab7 = drive.actionBuilder(pose7)
                .waitSeconds(1)
                .setTangent(Math.toRadians(0)) // Tangent aligns with leftward X movement
                .lineToX(-41) // Move along the X-axis to -43
                .waitSeconds(0.25)
                .turn(Math.toRadians(180))
                .waitSeconds(0.25)
                .setTangent(270)
                .lineToY(-69.75)
                .waitSeconds(0.25);

        TrajectoryActionBuilder tab8 = drive.actionBuilder(pose8)
                .waitSeconds(1)
                .setTangent(Math.toRadians(90))
                .lineToY(-63)
                .waitSeconds(0.25)
                .setTangent(Math.toRadians(180))
                .lineToX(11.5)
                .waitSeconds(0.25)
                .turn(Math.toRadians(180));


        TrajectoryActionBuilder tab9 = drive.actionBuilder(pose9)
                .waitSeconds(1)
                .setTangent(Math.toRadians(270))
                .lineToY(-41);

        TrajectoryActionBuilder tab10 = drive.actionBuilder(pose10)
                .waitSeconds(1)
                .setTangent(Math.toRadians(90)) // Tangent aligns with upward Y movement
                .lineToY(lineToYSplineHeading2);



        waitForStart();

        if (opModeIsActive()) {
            Actions.runBlocking(
                    new SequentialAction(
                            everythingElse.initEverythingElse(),
                            claw.closeClaw(),
                            outakeServos.scoreOutake(),
                            dr4BMove.liftUp(),
                            tab1.build(),
                            claw.openClaw(),
                            tab2.build(),
                            dr4BMove.liftDown(),
                            outakeServos.getSpecimenOutake(),
                            tab3.build(),
                            claw.closeClaw(),
                            tab4.build(),
                            outakeServos.scoreOutake(),
                            dr4BMove.liftUp(),
                            tab5.build(),
                            claw.openClaw(),
                            tab6.build(),
                            dr4BMove.liftDown(),
                            outakeServos.getSpecimenOutake(),
                            tab7.build(),
                            claw.closeClaw(),
                            tab8.build(),
                            outakeServos.scoreOutake(),
                            dr4BMove.liftUp(),
                            tab9.build(),
                            claw.openClaw(),
                            tab10.build(),
                            outakeServos.getSpecimenOutake(),
                            dr4BMove.liftDown()
                        )
            );
        }

        while (opModeIsActive()) {

            dr4BMove.DR4B(fourBarPos);

        }
    }
}
