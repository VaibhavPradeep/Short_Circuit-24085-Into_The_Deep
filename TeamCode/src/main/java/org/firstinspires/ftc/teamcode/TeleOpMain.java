

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name = "TeleOpMain")
public class TeleOpMain extends OpMode {
    MecanumDriveCircuit MD = new MecanumDriveCircuit();
    IntakeLiftCamera ILC = new IntakeLiftCamera();

    @Override
    public void init() {
        MD.initDrive(hardwareMap);
        ILC.initILC(hardwareMap);
    }

    @Override
    public void loop() {
        // mecanum drive
        MD.driveMecanum(gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x);

        // moive dr4b motors, left and right triggers (up and down repectively), amount pressed is speed
        ILC.moveDR4BMotors(gamepad2.left_trigger, gamepad2.right_trigger);

        // move intake servo, b is stop, x is intake, a is outake
        ILC.moveIntakeServo(gamepad1.a, gamepad1.x, gamepad1.b);

        // specimen servo, left bumper opens it, right closes it
        ILC.moveSpecimenServo(gamepad2.left_bumper, gamepad2.right_bumper);

        // moves arm servos, y is up, a is down
        ILC.moveArmServos(gamepad2.y, gamepad2.a);

        // helps with arm positions
        ILC.setArmZero(gamepad1.x);

        // updates telemetry
        ILC.addTelemetry(telemetry);

    }
}
