package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

@Config
@TeleOp(name = "Hood Servo Tuner")
public class HoodServoTuner extends OpMode {

    private Servo hoodServo;

    // Tunables in Dashboard
    public static double hoodPos = 0.5;   // 0.0–1.0
    public static double step = 0.01;     // how much D-pad up/down changes the position

    // Optional helper variable you can use to log "distance" from goal
    public static double testDistanceMeters = 3.0;

    private boolean prevUp = false;
    private boolean prevDown = false;

    @Override
    public void init() {
        hoodServo = hardwareMap.get(Servo.class, "hoodServo");
        hoodServo.setPosition(hoodPos);
    }

    @Override
    public void loop() {
        boolean dpadUp = gamepad1.dpad_up;
        boolean dpadDown = gamepad1.dpad_down;

        boolean upJustPressed = dpadUp && !prevUp;
        boolean downJustPressed = dpadDown && !prevDown;

        if (upJustPressed) {
            hoodPos += step;
        }
        if (downJustPressed) {
            hoodPos -= step;
        }

        hoodPos = Range.clip(hoodPos, 0.0, 1.0);
        hoodServo.setPosition(hoodPos);

        telemetry.addLine("=== Hood Servo Tuner ===");
        telemetry.addData("Hood Pos", "%.3f", hoodPos);
        telemetry.addData("Step", "%.3f", step);
        telemetry.addData("Test Distance (m)", "%.2f", testDistanceMeters);
        telemetry.addLine("Controls: Dpad Up/Down to tweak, Dashboard to fine-tune");
        telemetry.update();

        prevUp = dpadUp;
        prevDown = dpadDown;
    }
}
