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






















                                //Zone 1

                               /* .lineToSplineHeading(new Pose2d(-38, -58, Math.toRadians(90)))

                                .splineTo(new Vector2d(-47, -48), Math.toRadians(160))

                                .lineToSplineHeading(new Pose2d(-37, -35, Math.toRadians(180)))

                                .lineToSplineHeading(new Pose2d(-45, -58, Math.toRadians(.0001)))

                                .lineToSplineHeading(new Pose2d(18, -58, Math.toRadians(0)))

                                .splineTo(new Vector2d(49, -28), Math.toRadians(0))
*/

                        //middle
                                /*.lineToSplineHeading(new Pose2d(-38, -58, Math.toRadians(90)))

                                .lineToSplineHeading(new Pose2d(-38, -33, Math.toRadians(90)))

                                .waitSeconds(1)

                               // .lineToSplineHeading(new Pose2d(-37, -35, Math.toRadians(180)))

                                .lineToSplineHeading(new Pose2d(-45, -58, Math.toRadians(.0001)))

                                .lineToSplineHeading(new Pose2d(18, -58, Math.toRadians(0)))

                                .splineTo(new Vector2d(49, -34), Math.toRadians(0))
*/



                                //right/zone6

                                .lineToSplineHeading(new Pose2d(-44, -49, Math.toRadians(90)))

                                .lineToSplineHeading(new Pose2d(-44, -30, Math.toRadians(90)))

                                // .waitSeconds(3)

                                // .lineToSplineHeading(new Pose2d(-41.1, -35.1, Math.toRadians(0)))

                                .turn(0)

                                // .waitSeconds(3)


                                .lineToSplineHeading(new Pose2d(-39, -30, Math.toRadians(0)))

                                //.addTemporalMarker(() -> {flicker.setPosition(0);} )

                                .waitSeconds(1)

                                .lineToSplineHeading(new Pose2d(-46, -30, Math.toRadians(0)))

                                .lineToSplineHeading(new Pose2d(-46, -60, Math.toRadians(-1)))//-10?

                                .lineToSplineHeading(new Pose2d(14, -58, Math.toRadians(0)))


                                .splineTo(new Vector2d(42, -29), Math.toRadians(0))







                                .build()
                );

        meepMeep.setBackground(MeepMeep.Background.FIELD_CENTERSTAGE_JUICE_LIGHT)
                .setDarkMode(false)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}