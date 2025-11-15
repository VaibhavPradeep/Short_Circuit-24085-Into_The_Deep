package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.hardware.bosch.BHI260IMU;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

@Config
//@TeleOp(name = "New PID for yaw auto")
public class PIDForYawAuto extends OpMode {

    DcMotor motor;
    public static double integralSum = 0;
    public static double kp = 0;
    public static double ki = 0;
    public static double kd = 0;
    public static double target = 0;
    public static double targetAngle = 90;
    double targetAngleRadians = Math.toRadians(targetAngle);

    BHI260IMU imu;
    BHI260IMU.Parameters parameters;
    ElapsedTime timer = new ElapsedTime();
    public double lastError = 0;

    // TODO: FIND OUT THESE VALUES
    static final double TICKS_PER_REV = 1440; // adjust to your motor
    static final double GEAR_RATIO = 1.0;     // adjust if geared
    static final double TICKS_TO_RADIANS = 2 * Math.PI / (TICKS_PER_REV * GEAR_RATIO);

    @Override
    public void init() {
        motor = hardwareMap.get(DcMotor.class, "rotationMotor");
        imu = hardwareMap.get(BHI260IMU.class, "imu");
        parameters = new BHI260IMU.Parameters(
                new RevHubOrientationOnRobot(
                        RevHubOrientationOnRobot.LogoFacingDirection.LEFT,
                        RevHubOrientationOnRobot.UsbFacingDirection.UP
                )
        );
        imu.initialize(parameters);
        imu.resetYaw();

        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        /*BHI260IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.UP,
                RevHubOrientationOnRobot.UsbFacingDirection.FORWARD
        ));
        imu.initialize(parameters);
        imu.resetYaw();

         */

    }

    @Override
    public void loop() {
        /* encoder values
        double power = PIDControl(target, motor.getCurrentPosition());
        motor.setPower(power);
        */
        /*
        YawPitchRollAngles orientation = imu.getRobotYawPitchRollAngles();
        double currentAngle = orientation.getYaw(AngleUnit.RADIANS);
        double turretRelativeAngle = motor.getCurrentPosition() * TICKS_TO_RADIANS;

        //  compute field-relative turret angle
        double turretAbsoluteAngle = turretRelativeAngle - currentAngle;

        // use turretAbsoluteAngle as state
        double power = PIDControl(targetAngleRadians, turretAbsoluteAngle);
         */

        YawPitchRollAngles orientation = imu.getRobotYawPitchRollAngles();
        double currentAngle = orientation.getYaw(AngleUnit.RADIANS);
        double power = PIDControl(targetAngleRadians, currentAngle);


        motor.setPower(Math.max(-1.0, Math.min(1.0, power)));

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        telemetry.addData("target", target);
        telemetry.addData("current", currentAngle);
        telemetry.update();

    }

    public double PIDControl(double reference, double state) {
        // IF CONTINUOUS
        // double error = angleWrap(reference - state);
        double error = reference - state;
        integralSum += error * timer.seconds();
        double derivitive = (error - lastError) / timer.seconds();
        lastError = error;



        timer.reset();

        double output = (error * kp) + (derivitive * kd) + (integralSum * ki);
        return output;
    }

    // IF CONTINUOUS

    /*
    public double angleWrap(double radians) {
        while (radians > Math.PI) {
            radians -= 2 * Math.PI;
        }
        while (radians < Math.PI) {
            radians += 2 * Math.PI;
        }
        return radians;
    }
     */

}
