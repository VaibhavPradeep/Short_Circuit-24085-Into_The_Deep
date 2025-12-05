package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.Servo;

//import org.firstinspires.ftc.teamcode.util.RobotPIDFConstants;

public class FeedServoHelper {

    private Servo servo;

    public FeedServoHelper(Servo s) {
        this.servo = s;
        rest();
    }

    public void fire() {
//        servo.setPosition(RobotPIDFConstants.FEED_FIRE_POS);
    }

    public void rest() {
//        servo.setPosition(RobotPIDFConstants.FEED_REST_POS);
    }
}
