package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;


@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name = "Run Motor Until Stop", group = "Examples")
public class RunMotorUntilStop extends OpMode {
    private DcMotor motor;

    @Override
    public void init() {
        motor = hardwareMap.get(DcMotor.class, "motor");
        motor.setDirection(DcMotor.Direction.FORWARD);
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motor.setPower(0);
    }

    @Override
    public void start() {
        motor.setPower(1.0);
    }

    @Override
    public void loop() {
        telemetry.addData("Motor Status", "Running");
        telemetry.addData("Motor Power", motor.getPower());
        telemetry.update();
    }

    @Override
    public void stop() {
        motor.setPower(0);
    }
}
