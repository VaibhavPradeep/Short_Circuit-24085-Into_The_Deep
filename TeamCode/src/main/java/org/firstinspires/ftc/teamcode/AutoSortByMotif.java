//package org.firstinspires.ftc.teamcode.autonomous;
//
//import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
//import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
//import com.qualcomm.robotcore.hardware.DcMotor;
//import com.qualcomm.robotcore.hardware.DcMotorEx;
//import com.qualcomm.robotcore.hardware.Servo;
//
////import org.firstinspires.ftc.teamcode.mechanisms.FeedServoHelper;
////import org.firstinspires.ftc.teamcode.sensors.HuskyLensI2C;
////import org.firstinspires.ftc.teamcode.sensors.HuskyLensI2C.ArtifactColor;
////import org.firstinspires.ftc.teamcode.sensors.HuskyLensI2C.MotifType;
////import org.firstinspires.ftc.teamcode.util.RobotPIDFConstants;
//
//@Autonomous(name="AutoSortByMotif", group="Auto")
//public class AutoSortByMotif extends LinearOpMode {
//
//    private DcMotorEx intake, shooter, spindexer;
////    private FeedServoHelper feeder;
//
////    private HuskyLensI2C motifCam;
////    private HuskyLensI2C sorterCam;
//
////    ArtifactColor[] pattern = new ArtifactColor[3];
//
//    @Override
//    public void runOpMode() throws InterruptedException {
//
//        intake = hardwareMap.get(DcMotorEx.class, "intake");
//        shooter = hardwareMap.get(DcMotorEx.class, "shooter");
//        spindexer = hardwareMap.get(DcMotorEx.class, "spindexer");
//        Servo feed = hardwareMap.get(Servo.class, "feeder");
//
////        feeder = new FeedServoHelper(feed);
//
//        intake.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        shooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//
//        spindexer.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        spindexer.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//
//        // ---- cameras ----
////        motifCam  = new HuskyLensI2C(hardwareMap, "motifCam");
////        sorterCam = new HuskyLensI2C(hardwareMap, "sorterCam");
//
//        telemetry.addLine("Ready");
//        telemetry.update();
//
//        waitForStart();
//
//        // ---- STEP 1: read motif ----
////        MotifType motif = detectMotif(2000);
////        buildPattern(motif);
//
//        // ---- STEP 2: start shooter + intake ----
////        intake.setPower(RobotPIDFConstants.INTAKE_POWER);
////        shooter.setPower(RobotPIDFConstants.SHOOTER_POWER);
//
//        // ---- STEP 3: do 3-shot sequence ----
//        for (int i = 0; i < 3; i++) {
////            ArtifactColor desired = pattern[i];
//
////            ArtifactColor seen = sorterCam.getArtifactColor();
//
//            int safety = 0;
//            while (seen != desired && safety < 5 && opModeIsActive()) {
//                rotateOneSlot();
//                sleep(RobotPIDFConstants.SLOT_SETTLE_MS);
//                seen = sorterCam.getArtifactColor();
//                safety++;
//            }
//
//            fireOne();
//        }
//
//        intake.setPower(0);
//        shooter.setPower(0);
//        spindexer.setPower(0);
//    }
//
//    private MotifType detectMotif(long timeout) {
//        long start = System.currentTimeMillis();
//        MotifType m = MotifType.UNKNOWN;
//
//        while (m == MotifType.UNKNOWN &&
//                System.currentTimeMillis() - start < timeout &&
//                opModeIsActive()) {
//
//            int id = motifCam.getAprilTagId();
//            m = motifCam.motifFromTagId(id);
//
//            telemetry.addData("Tag", id);
//            telemetry.addData("Motif", m);
//            telemetry.update();
//        }
//
//        return m;
//    }
//
//    private void buildPattern(MotifType m) {
//        switch (m) {
//            case GPP:
//                pattern[0] = ArtifactColor.GREEN;
//                pattern[1] = ArtifactColor.PURPLE;
//                pattern[2] = ArtifactColor.PURPLE;
//                break;
//            case PGP:
//                pattern[0] = ArtifactColor.PURPLE;
//                pattern[1] = ArtifactColor.GREEN;
//                pattern[2] = ArtifactColor.PURPLE;
//                break;
//            case PPG:
//                pattern[0] = ArtifactColor.PURPLE;
//                pattern[1] = ArtifactColor.PURPLE;
//                pattern[2] = ArtifactColor.GREEN;
//                break;
//            default:
//                pattern[0] = pattern[1] = pattern[2] = ArtifactColor.UNKNOWN;
//        }
//    }
//
//    private void rotateOneSlot() {
//        int target = spindexer.getCurrentPosition() + RobotPIDFConstants.TICKS_PER_SLOT;
//
//        spindexer.setTargetPosition(target);
//        spindexer.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        spindexer.setPower(RobotPIDFConstants.SPINDEXER_POWER);
//
//        while (opModeIsActive() && spindexer.isBusy()) {}
//
//        spindexer.setPower(0);
//        spindexer.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//    }
//
//    private void fireOne() throws InterruptedException {
//        feeder.fire();
//        sleep(RobotPIDFConstants.SHOOT_TIME_MS);
//        feeder.rest();
//
//        rotateOneSlot();
//        sleep(RobotPIDFConstants.SLOT_SETTLE_MS);
//    }
//}
