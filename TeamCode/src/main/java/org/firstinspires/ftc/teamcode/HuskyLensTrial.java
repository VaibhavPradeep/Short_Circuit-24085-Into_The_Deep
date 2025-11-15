package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.dfrobot.HuskyLens;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.internal.system.Deadline;

import java.util.concurrent.TimeUnit;

//@TeleOp(name = "HuskyLensTrial_Iterative")
public class HuskyLensTrial extends OpMode {
    /* tag id 1: GPP
    2: PGP
    3: PPG
     */
    private final int READ_PERIOD = 1;

    private HuskyLens huskyLens;
    private Deadline rateLimit;

    boolean motif1 = false;
    boolean motif2 = false;
    boolean motif3 = false;

    @Override
    public void init() {
        huskyLens = hardwareMap.get(HuskyLens.class, "huskylens");

        // Rate limiter for telemetry
        rateLimit = new Deadline(READ_PERIOD, TimeUnit.SECONDS);
        rateLimit.expire();

        if (!huskyLens.knock()) {
            telemetry.addData(">>", "Problem communicating with " + huskyLens.getDeviceName());
        } else {
            telemetry.addData(">>", "Initialized successfully. Ready to start!");
        }

        // Choose the algorithm
        huskyLens.selectAlgorithm(HuskyLens.Algorithm.TAG_RECOGNITION);
        // huskyLens.selectAlgorithm(HuskyLens.Algorithm.COLOR_RECOGNITION);
    }

    @Override
    public void loop() {
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
            } else if (blocks[i].id == 2) {
                motif2 = true;
            } else if (blocks[i].id == 3) {
                motif3 = true;
            }
            // Example of accessing block fields:
            // blocks[i].x, blocks[i].y, blocks[i].width, blocks[i].height, blocks[i].id
        }

        telemetry.update();
    }
}
