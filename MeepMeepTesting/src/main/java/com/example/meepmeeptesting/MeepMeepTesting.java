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
                        drive.trajectorySequenceBuilder(new Pose2d(14.5, -60, Math.toRadians(90)))//-38, 60 for blue right

//-38, 60 for blue audiance side




                                .splineTo(new Vector2d(13, -30), Math.toRadians(90))

                                .waitSeconds(1)
                                //place purple
                                .waitSeconds(1)

                                .lineToSplineHeading(new Pose2d(13, -46, Math.toRadians(45)))
                                .waitSeconds(.1)

                                //raise lift

                                .splineTo(new Vector2d(51, -34), Math.toRadians(0))//slow to board, middle


                                .waitSeconds(1)
                                //score


                                .waitSeconds(1)
                                .setReversed(true)



                                // .splineTo(new Vector2d(50, 40), Math.toRadians(0))

                                .splineTo(new Vector2d(45, -59), Math.toRadians(-90))
                                .waitSeconds(1)

                                .build()


                );

        meepMeep.setBackground(MeepMeep.Background.FIELD_CENTERSTAGE_JUICE_LIGHT)
                .setDarkMode(false)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}