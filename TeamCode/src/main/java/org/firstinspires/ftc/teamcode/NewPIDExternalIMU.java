package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.hardware.bosch.BHI260IMU;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.hardware.dfrobot.HuskyLens;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.robotcore.internal.system.Deadline;

import java.util.Locale;
import java.util.concurrent.TimeUnit;

@Config
@TeleOp(name = "NewPID external imu")
public class NewPIDExternalIMU extends OpMode {


    final int READ_PERIOD = 1;

    // TODO: find lever pos
    double leverPos = 0.5;
    DcMotor intakeMotor;
    Servo pitchServo;
    DcMotor rotationMotor;
    DcMotor shootingMotor;

    DcMotor transferMotor;
    CRServo sorterServo;
    Servo leverServo;

    ColorSensor colorSensor;
    HuskyLens huskyLens;
    Deadline rateLimit;
    public static double integralSum = 0;
    public static double kp = 0;
    public static double ki = 0;
    public static double kd = 0;
    public static double target = 0;
    public static double targetAngle = 90;
    double targetAngleRadians = Math.toRadians(targetAngle);
    Orientation angles;

    Acceleration gravity;
    double yawOffset = 0;

    BNO055IMU turretImu;
    ElapsedTime timer = new ElapsedTime();
    public double lastError = 0;

    // TODO: FIND OUT THESE VALUES
    static final double TICKS_PER_REV = 1440; // adjust to your motor
    static final double GEAR_RATIO = 1.0;     // adjust if geared
    static final double TICKS_TO_RADIANS = 2 * Math.PI / (TICKS_PER_REV * GEAR_RATIO);

    @Override
    public void init() {
        /*

        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.loggingEnabled = true;

        turretImu.initialize(parameters);
        angles   = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
         */

        // Set up the parameters with which we will use our IMU. Note that integration
        // algorithm here just reports accelerations to the logcat log; it doesn't actually
        // provide positional information.
        turretImu = hardwareMap.get(BNO055IMU.class, "turretImu");
        intakeMotor = hardwareMap.get(DcMotor.class, "intakeMotor");
        pitchServo = hardwareMap.get(Servo.class,"pitchServo");
        rotationMotor = hardwareMap.get(DcMotor.class, "rotationMotor");
        shootingMotor = hardwareMap.get(DcMotor.class, "shootingMotor");
        colorSensor = hardwareMap.get(ColorSensor.class,"colorSensor");
        huskyLens = hardwareMap.get(HuskyLens.class, "huskylens");

        transferMotor = hardwareMap.get(DcMotor.class, "transferMotor");
        sorterServo = hardwareMap.get(CRServo.class, "sorterServo");
        leverServo = hardwareMap.get(Servo.class,"leverServo");

        rotationMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rotationMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Rate limiter for telemetry
        rateLimit = new Deadline(READ_PERIOD, TimeUnit.SECONDS);
        rateLimit.expire();

        leverServo.setPosition(0);

        // Choose the algorithm
        huskyLens.selectAlgorithm(HuskyLens.Algorithm.TAG_RECOGNITION);
        // huskyLens.selectAlgorithm(HuskyLens.Algorithm.COLOR_RECOGNITION);


        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit           = BNO055IMU.AngleUnit.RADIANS;
        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;

        // Retrieve and initialize the IMU. We expect the IMU to be attached to an I2C port
        // on a Core Device Interface Module, configured to be a sensor of type "AdaFruit IMU",
        // and named "imu".
        turretImu.initialize(parameters);

        // Set up our telemetry dashboard
        turretImu.startAccelerationIntegration(new Position(), new Velocity(), 1000);

        angles = turretImu.getAngularOrientation(
                AxesReference.INTRINSIC,
                AxesOrder.ZYX,
                AngleUnit.RADIANS
        );

        yawOffset = angles.firstAngle;
        timer.reset();
        integralSum = 0;
        lastError = 0;

        /*
        parameters = new BHI260IMU.Parameters(
                new RevHubOrientationOnRobot(
                        RevHubOrientationOnRobot.LogoFacingDirection.LEFT,
                        RevHubOrientationOnRobot.UsbFacingDirection.UP
                )
        );
        imu.initialize(parameters);
        imu.resetYaw();
         */
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

        /*
        YawPitchRollAngles orientation = imu.getRobotYawPitchRollAngles();
        double currentAngle = orientation.getYaw(AngleUnit.RADIANS);
        double power = PIDControl(targetAngleRadians, currentAngle);

         */
        angles   = turretImu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.RADIANS);
        gravity  = turretImu.getGravity();

        double currentYaw = angles.firstAngle;
        double power = PIDControl(targetAngleRadians, currentYaw);

        rotationMotor.setPower(Math.max(-1.0, Math.min(1.0, power)));

        /*
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        telemetry.addData("target", target);
        telemetry.addData("current", currentAngle);
        telemetry.update();

         */

    }

    public double PIDControl(double reference, double state) {
        // IF CONTINUOUS
        // double error = angleWrap(reference - state);
        double error = reference - state;
        integralSum += error * timer.seconds();
        double derivative = (error - lastError) / timer.seconds();
        lastError = error;



        timer.reset();

        double output = (error * kp) + (derivative * kd) + (integralSum * ki);
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
