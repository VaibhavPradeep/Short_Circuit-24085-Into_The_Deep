package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.PIDFCoefficients;

public class RobotPIDFConstants {

    public static int TICKS_PER_REV = 537;
    public static int SPINDEXER_SLOTS = 3;
    public static int TICKS_PER_SLOT = TICKS_PER_REV / SPINDEXER_SLOTS;

    public static PIDFCoefficients INTAKE_PIDF =
            new PIDFCoefficients(1.0, 0.0, 0.0, 0.0);

    public static PIDFCoefficients SHOOTER_PIDF =
            new PIDFCoefficients(1.0, 0.0, 0.0, 0.0);

    public static PIDFCoefficients SPINDEX_PIDF =
            new PIDFCoefficients(1.0, 0.0, 0.0, 0.0);

    public static double INTAKE_POWER = 0.7;
    public static double SHOOTER_POWER = 1.0;
    public static double SPINDEXER_POWER = 0.5;

    public static long SHOOT_TIME_MS = 350;
    public static long SLOT_SETTLE_MS = 120;

    public static double FEED_REST_POS = 0.0;
    public static double FEED_FIRE_POS = 0.55;
}
