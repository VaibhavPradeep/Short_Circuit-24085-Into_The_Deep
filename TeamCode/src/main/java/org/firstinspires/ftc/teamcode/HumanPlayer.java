package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
@Autonomous( name = "human")
public class HumanPlayer extends OpMode {

    Drivetrain auto = new Drivetrain();

    final double speed = 0.3;

    final int driveToRack = 5;

    final double turnAroundForSpecimen = 180;

    @Override
    public void init() {
        auto.initDrivetrain(hardwareMap);
    }

    @Override
    public void start() {
        auto.strafe("right", 0.3, 50);
    }

    @Override
    public void loop() {

    }
}
