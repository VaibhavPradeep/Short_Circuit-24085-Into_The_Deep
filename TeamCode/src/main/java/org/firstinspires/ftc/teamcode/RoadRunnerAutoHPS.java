/*
package org.firstinspires.ftc.teamcode.autonomous;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.MecanumDriveCircuit;

@Autonomous(name = "Reyautonomous", group = "Autonomous")
public class Reyautism extends LinearOpMode {
    @Override
    public void runOpMode() {
        // Initialize RoadRunner drive
        MecanumDrive drive = new MecanumDrive(hardwareMap);

        DcMotor dr4bMotor1 = hardwareMap.get(DcMotor.class, "dr4bMotor1");
        DcMotor dr4bMotor2 = hardwareMap.get(DcMotor.class, "dr4bMotor2"); // This motor is reversed
        Servo leftArmServo = hardwareMap.get(Servo.class, "leftArmServo");
        Servo rightArmServo = hardwareMap.get(Servo.class, "rightArmServo");
        Servo clawServo = hardwareMap.get(Servo.class, "clawServo");

        dr4bMotor2.setDirection(DcMotor.Direction.REVERSE);

        clawServo.setPosition(0.5); // Assume 0.5 is the closed position
        leftArmServo.setPosition(0);
        rightArmServo.setPosition(0);

        Pose2d startPose = new Pose2d(0, 0, Math.toRadians(0));
        drive.setPoseEstimate(startPose);

        Trajectory moveForward2Feet = drive.trajectoryBuilder(startPose)
                .forward(24) // 2 feet = 24 inches
                .build();

        Trajectory moveForward2Inches = drive.trajectoryBuilder(moveForward2Feet.end())
                .forward(2) // 2 inches
                .build();

        waitForStart();

        if (opModeIsActive()) {
            drive.followTrajectory(moveForward2Feet);

            dr4bMotor1.setPower(0.5); // Adjust power as needed
            dr4bMotor2.setPower(0.5); // Both motors run in sync
            sleep(1000); // Adjust duration to raise to desired height
            dr4bMotor1.setPower(0); // Stop both motors
            dr4bMotor2.setPower(0);

            drive.followTrajectory(moveForward2Inches);

            leftArmServo.setPosition(1.0); // Assume 1.0 rotates the left arm 180 degrees
            rightArmServo.setPosition(1.0); // Assume 1.0 rotates the right arm 180 degrees
            sleep(500); // Wait for the servos to complete rotation

            clawServo.setPosition(0.0); // Assume 0.0 opens the claw
            sleep(500); // Wait for the claw to fully open


        }
    }
}

 */
