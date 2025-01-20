package org.firstinspires.ftc.teamcode;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

@Autonomous(name = "Newest RR Code")
public class NewestRRCode extends OpMode {

    // Configurable variables
    public static double startX = 9.5; // Starting X position
    public static int DR4B_TICKS = 0;
    public static double startY = -72; // Starting Y position
    public static double startHeading = 270; // Starting heading in degrees
    public static double lineToYSplineHeading1 = -43; // Target Y position for spline heading
    public static double lineToYSplineHeading2 = -51;
    public static double lineToXSplineHeading3 = 36;
    public static double lineToYSplineHeading4 = -4;
    public static double strafeToX5 = 48;
    public static double strafeToY5 = -4;
    public static double lineToYSplineHeading6 = -65;
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
                leftOutakeServo.setPosition(0.35);
                rightOutakeServo.setPosition(0.35);
                return false;
            }
        }
        public Action initOutake() {
            return new OutakeServos.InitOutake();
        }

        public class GetSpecimenOutake implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                leftOutakeServo.setPosition(0.8);
                rightOutakeServo.setPosition(0.8);
                return false;
            }
        }
        public Action getSpecimenOutake() {
            return new OutakeServos.GetSpecimenOutake();
        }

        public class ScoreOutake implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                leftOutakeServo.setPosition(0.45);
                rightOutakeServo.setPosition(0.45);
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
        }

        public void DR4B(int ticks) {
            leftDR4BMotor.setTargetPosition(ticks);
            rightDR4BMotor.setTargetPosition(ticks);
            leftDR4BMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rightDR4BMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            leftDR4BMotor.setPower(1);
            rightDR4BMotor.setPower(1);

            // Wait until the motors reach the target position
            if (leftDR4BMotor.getCurrentPosition() == ticks && rightDR4BMotor.getCurrentPosition() == ticks) {
                leftDR4BMotor.setPower(0);
                rightDR4BMotor.setPower(0);
            }

            // Stop the motors once the target position is reached
            leftDR4BMotor.setPower(0);
            rightDR4BMotor.setPower(0);
        }
        public void HoldPosition() {
            leftDR4BMotor.setTargetPosition(fourBarPos);
            rightDR4BMotor.setTargetPosition(fourBarPos);
        }
        public class LiftUp implements Action {
            public LiftUp(){
                super();
            }

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                fourBarPos = 950;
                return false;
            }
        }
        public Action liftUp() {
            return new DR4BMove.LiftUp();
        }
        public class LiftDown implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                fourBarPos = -950;
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
    }

    Pose2d initialPose = new Pose2d(startX, startY, Math.toRadians(startHeading));
    Pose2d nextPose = new Pose2d(startX, -43, Math.toRadians(startHeading));
    MecanumDrive drive;
    Claw claw;
    OutakeServos outakeServos;
    DR4BMove dr4BMove;
    EverythingElse everythingElse;

    @Override
    public void init() {
        drive = new MecanumDrive(hardwareMap, initialPose);
        claw = new Claw(hardwareMap);
        outakeServos = new OutakeServos(hardwareMap);
        dr4BMove = new DR4BMove(hardwareMap);
        everythingElse = new EverythingElse(hardwareMap);
    }

    @Override
    public void start() {
        TrajectoryActionBuilder tab1 = drive.actionBuilder(initialPose)
                .waitSeconds(3)
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
        //Action trajectoryActionCloseOut = tab2.endTrajectory().fresh().build();

        Actions.runBlocking(
                new SequentialAction(
                        claw.closeClaw(),
                        outakeServos.scoreOutake(),
                        dr4BMove.liftUp(),
                        tab1.build(),
                        //claw.openClaw(),
                        tab2.build()
                )
        );
    }

    @Override
    public void loop() {
        dr4BMove.DR4B(fourBarPos);
        telemetry.addData("Trying to get to", fourBarPos);
        telemetry.addData("Actually At", dr4BMove.leftDR4BMotor.getCurrentPosition());
        telemetry.update();
    }
}
