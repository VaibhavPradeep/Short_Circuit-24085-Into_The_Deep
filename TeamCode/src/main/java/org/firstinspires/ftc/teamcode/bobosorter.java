package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.hardware.dfrobot.HuskyLens;

@TeleOp(name = "HuskyLens Purple Green Detect", group = "Test")
public class bobosorter extends LinearOpMode {

    HuskyLens huskyLens;

    @Override
    public void runOpMode() {

        huskyLens = hardwareMap.get(HuskyLens.class, "huskylens");

        telemetry.addLine("Checking HuskyLens...");
        telemetry.update();

        if (!huskyLens.knock()) {
            telemetry.addLine("ERROR: HuskyLens not detected!");
            telemetry.update();
            sleep(3000);
            return;
        }

        telemetry.addLine("HuskyLens Ready");
        telemetry.addLine("Mode: COLOR RECOGNITION");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {

            HuskyLens.Block[] blocks = huskyLens.blocks();

            boolean purpleDetected = false;
            boolean greenDetected = false;

            for (HuskyLens.Block block : blocks) {

                // filter out tiny detections
                if (block.width * block.height < 3000) continue;

                if (block.id == 1) purpleDetected = true;
                if (block.id == 2) greenDetected = true;
            }

            telemetry.addData("Purple Ball", purpleDetected);
            telemetry.addData("Green Ball", greenDetected);
            telemetry.addData("Blocks Seen", blocks.length);
            telemetry.update();
        }
    }
}
