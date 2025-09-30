package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.dfrobot.HuskyLens;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.internal.system.Deadline;

import java.util.concurrent.TimeUnit;

@TeleOp(name = "TeleOpDecodeInitial")
public class TeleOpDecodeInitial extends OpMode {

    MecanumDriveCircuit drivetrain = new MecanumDriveCircuit();
    IntakeLiftCameraDecode ILC = new IntakeLiftCameraDecode();

    private final int READ_PERIOD = 1;
    boolean motif1 = false;
    boolean motif2 = false;
    boolean motif3 = false;

    private HuskyLens huskyLens;
    private Deadline rateLimit;

    /*
    FINISHED: DT: 4 motors
    TODO: 1 motor for itnake
    TODO: 1 CR servo for sorter
    TODO: 1 servo for transfer wheels

    TODO: Turret: 3 Motors
    TODO: - one for pitch, one for rotation, one for shooting (wheels)


     */

    @Override
    public void init() {
        drivetrain.initDrive(hardwareMap);
        ILC.initILC(hardwareMap);

        huskyLens = hardwareMap.get(HuskyLens.class, "huskylens");

        // Rate limiter for telemetry
        rateLimit = new Deadline(READ_PERIOD, TimeUnit.SECONDS);
        rateLimit.expire();

        // Choose the algorithm
        huskyLens.selectAlgorithm(HuskyLens.Algorithm.TAG_RECOGNITION);
        // huskyLens.selectAlgorithm(HuskyLens.Algorithm.COLOR_RECOGNITION);

    }

    @Override
    public void loop() {
        drivetrain.driveMecanum(gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x);

        if (gamepad2.dpad_left) {
            ILC.LateralTurretManualMove("left");
        }
        if (gamepad2.dpad_right) {
            ILC.LateralTurretManualMove("right");
        }
        if (gamepad2.dpad_up) {
            ILC.VerticalTurretManualMove("up");
        }
        if (gamepad2.dpad_down) {
            ILC.VerticalTurretManualMove("down");
        }

        if (gamepad2.a) {
            ILC.ShootingOn();
        }
        if (gamepad2.b) {
            ILC.ShootingOff();
        }

        if (gamepad2.left_bumper) {
            ILC.SorterOn();
        }
        if (gamepad2.right_bumper) {
            ILC.SorterOff();
        }

        if (gamepad2.x) {
            ILC.TransferOn();
        }
        if (gamepad2.y) {
            ILC.TransferOff();
        }

        if (gamepad1.left_bumper) {
            ILC.IntakeOn();
        }
        if (gamepad1.right_bumper) {
            ILC.IntakeOff();
        }

        if (!rateLimit.hasExpired()) {
            return;
        }
        rateLimit.reset();

        // Get blocks (recognized objects)
        HuskyLens.Block[] blocks = huskyLens.blocks();
        telemetry.addData("Block count", blocks.length);

        for (int i = 0; i < blocks.length; i++) {
            telemetry.addData("Block", blocks[i].toString());
            if (blocks[i].id == 1) {
                motif1 = true;
                gamepad1.rumble(500);
                gamepad2.rumble(500);
            } else if (blocks[i].id == 2) {
                motif2 = true;
                gamepad1.rumble(500);
                gamepad2.rumble(500);
            } else if (blocks[i].id == 3) {
                motif3 = true;
                gamepad1.rumble(500);
                gamepad2.rumble(500);
            }
        }

        telemetry.update();
    }
}
