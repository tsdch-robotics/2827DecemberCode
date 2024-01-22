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
                        drive.trajectorySequenceBuilder(new Pose2d(-33, -60, Math.toRadians(90)))//-38, 60 for blue right

//-38, 60 for blue audiance side





                                //initialize servos
                                .lineToSplineHeading(new Pose2d(-42.5, -51.4, Math.toRadians(90)))//54 or 51?a

                                .splineTo(new Vector2d(-46.71, -36.71), Math.toRadians(90))
                                .lineToSplineHeading(new Pose2d (-46.7, -36.7, Math.toRadians(90)))//ensure 90
                                .waitSeconds(1)
                                //place pixel


                                .lineToSplineHeading(new Pose2d (-46.7, -44.7, Math.toRadians(90)))//back off


                                .lineToSplineHeading(new Pose2d (-43.5, -58.9, Math.toRadians(0)))//allign with wall


                                .lineToSplineHeading(new Pose2d(13.1, -58.7, Math.toRadians(0)))//out of truss
                                //raise lift

                                .splineTo(new Vector2d(44.5, -29.6), Math.toRadians(0))

                                .splineTo(new Vector2d(51.2, -29.6), Math.toRadians(0))//slow to board

                                .setReversed(true)
                                .splineTo(new Vector2d(46.5, -59.8), Math.toRadians(-90))//slow to park

                                .build()
                );

        meepMeep.setBackground(MeepMeep.Background.FIELD_CENTERSTAGE_OFFICIAL)
                .setDarkMode(false)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}