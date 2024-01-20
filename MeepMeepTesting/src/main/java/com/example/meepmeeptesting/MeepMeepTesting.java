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






                                .lineToSplineHeading(new Pose2d(-40.9, -51.8, Math.toRadians(90)))



                                .splineTo(new Vector2d(-32.4, -34.3), Math.toRadians(0))
                                .lineToSplineHeading(new Pose2d(-32.41, -34.31, Math.toRadians(0)))


                                .waitSeconds(1)

                                .lineToSplineHeading(new Pose2d(-38.5, -34.3, Math.toRadians(0)))

                                .lineToSplineHeading(new Pose2d(-36.9, -58.7, Math.toRadians(0)))

                                .lineToSplineHeading(new Pose2d(13.1, -58.7, Math.toRadians(0)))


                                .splineTo(new Vector2d(44.5, -40.5), Math.toRadians(0))

                                .splineTo(new Vector2d(51.2, -40.5), Math.toRadians(0))//slow

                                .setReversed(true)
                                .splineTo(new Vector2d(46.5, -59.8), Math.toRadians(-90))//slow




                                .build()
                );

        meepMeep.setBackground(MeepMeep.Background.FIELD_CENTERSTAGE_JUICE_LIGHT)
                .setDarkMode(false)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}