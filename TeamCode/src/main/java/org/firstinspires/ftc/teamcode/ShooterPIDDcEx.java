package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

@Config
@TeleOp(name = "shooter PID Dc Ex")
public class ShooterPIDDcEx extends OpMode {
    private DcMotorEx shootingMotor;
    private Servo pitchServo;


    public static double leverPos = 0;

    ElapsedTime timer = new ElapsedTime();
    public double lastError = 0;
    public static double integralSum = 0;

    public static double kPs = 0.0;
    public static double kIs = 0.0;
    public static double kDs = 0.0;

    //private PIDController controller;

    public static double pitchPos = 0;
    public static double target = 0;

    private double maxV = 2800; // for 6000 rpm motor
    private double Kf = 32767 / maxV;
    private double Kp = Kf * 0.1;
    private double Ki = Kp * 0.1;
    private double Kd = 0;

    Servo leverServo;

    @Override
    public void init() {
        shootingMotor = hardwareMap.get(DcMotorEx.class, "shootingMotor");
        pitchServo = hardwareMap.get(Servo.class, "pitchServo");
        leverServo = hardwareMap.get(Servo.class, "leverServo");
        leverServo.setPosition(0);

        //controller = new PIDController(kPs,kIs,kDs);


        shootingMotor.setVelocityPIDFCoefficients(Kp, Ki, Kd, Kf);

        pitchServo.setDirection(Servo.Direction.REVERSE);
        pitchServo.setPosition(0);

        shootingMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        shootingMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        shootingMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
    }

    @Override
    public void start() {
        pitchServo.setPosition(0.3);
    }

    @Override
    public void loop() {
        //controller.setPID(kPs,kIs,kDs);
        //double pidOutput = controller.calculate(shootingMotor.getVelocity(),target);
        //shootingMotor.setPower(pidOutput);

        pitchServo.setPosition(pitchPos);
        // 1675 LONG
        leverServo.setPosition(leverPos);

        shootingMotor.setVelocity(target);
        //double power = PIDControl(target, shootingMotor.getVelocity());
        //shootingMotor.setPower(power);

        //telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        //telemetry.addData("Pos: ", shootingMotor.getVelocity());
        //telemetry.addData("target: ", target);
        //telemetry.update();
    }

    public double PIDControl(double reference, double state) {
        double error = reference - state;
        integralSum += error * timer.seconds();
        double derivative = (error - lastError) / timer.seconds();
        lastError = error;

        timer.reset();

        double output = (error * kPs) + (derivative * kDs) + (integralSum * kIs);
        return output;
    }

}