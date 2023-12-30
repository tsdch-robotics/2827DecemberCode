package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class MeepMeepTesting {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(800);

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(60, 52, Math.toRadians(180), Math.toRadians(180), 14.9)
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(new Pose2d(-38, 60, Math.toRadians(270)))
                                .splineTo(new Vector2d(-33, 37), Math.toRadians(-85))
                                .lineToSplineHeading(new Pose2d(-40, 57, Math.toRadians(0)))
                                .lineToSplineHeading(new Pose2d(20, 57,Math.toRadians(0)))
                                .splineTo(new Vector2d(45, 30), Math.toRadians(0))
                                //.lineToSplineHeading(new Pose2d(45, 60, Math.toRadians(-90)))


                                .lineToSplineHeading(new Pose2d(30, 10, Math.toRadians(0)))
                                .lineToSplineHeading(new Pose2d(-60, 10, Math.toRadians(0)))
                                .lineToSplineHeading(new Pose2d(-60, 18, Math.toRadians(0)))


                                //.splineTo(new Vector2d(45, 40), Math.toRadians(90))


                                .build()
                );

        meepMeep.setBackground(MeepMeep.Background.FIELD_CENTERSTAGE_JUICE_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}