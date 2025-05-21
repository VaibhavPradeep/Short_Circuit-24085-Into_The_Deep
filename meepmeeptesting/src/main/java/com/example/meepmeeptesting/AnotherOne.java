package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class AnotherOne {
    public static double startX = 9.5; // Starting X position
    public static double startY = -72; // Starting Y position
    public static double startHeading = 270; // Starting heading in degrees
    public static double lineToYSplineHeading1 = -41; // Target Y position for spline heading
    public static double lineToYSplineHeading2 = -51;
    public static double lineToXSplineHeading3 = 36;
    public static double lineToYSplineHeading4 = -20;
    public static double strafeToX5 = 48;
    public static double strafeToY5 = -20;
    public static double lineToYSplineHeading6 = -67;
    public static double turnAngle7 = 180;
    public static int fourBarPos = 0;

    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(600);

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 15.5)
                .build();

        myBot.runAction(myBot.getDrive().actionBuilder(new Pose2d(9.7, -72, Math.toRadians(270)))
                .waitSeconds(3)
                .setTangent(Math.toRadians(270)) // Tangent aligns with downward Y movement
                .lineToY(lineToYSplineHeading1) // Valid for downward Y movement
                .waitSeconds(4)
                .waitSeconds(1)
                .setTangent(Math.toRadians(90)) // Tangent aligns with upward Y movement
                .lineToY(lineToYSplineHeading2)
                .waitSeconds(1)
                .setTangent(Math.toRadians(0)) // Tangent aligns with rightward X movement
                .lineToX(lineToXSplineHeading3)// Valid for rightward X movement
                .waitSeconds(0.25)
                .setTangent(Math.toRadians(270))
                .lineToY(lineToYSplineHeading4)// Valid for downward Y movement
                .waitSeconds(0.25)
                .setTangent(Math.toRadians(0)) // Tangent aligns with rightward X movement
                .strafeTo(new Vector2d(strafeToX5, strafeToY5)) // Valid for simultaneous X-Y movement
                .waitSeconds(0.25)
                .setTangent(Math.toRadians(90)) // Tangent aligns with downward Y movement
                .lineToY(lineToYSplineHeading6)
                .waitSeconds(0.25)
                .lineToY(-20)
                .setTangent(Math.toRadians(0)) // Tangent aligns with rightward X movement
                .strafeTo(new Vector2d(60, strafeToY5))
                .setTangent(90)
                .lineToY(-67)
                .waitSeconds(0.25)
                .lineToY(-20)
                .setTangent(0)
                .strafeTo(new Vector2d(64.5, strafeToY5))
                .setTangent(90)
                .lineToY(-67)
                .build());

        meepMeep.setBackground(MeepMeep.Background.FIELD_INTO_THE_DEEP_JUICE_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}
