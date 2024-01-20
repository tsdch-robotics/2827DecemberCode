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




                               

                                .lineToSplineHeading(new Pose2d(-49, -49, Math.toRadians(90)))

                                .lineToSplineHeading(new Pose2d(-70, -30, Math.toRadians(90)))


                                // .lineToSplineHeading(new Pose2d(-52, -40, Math.toRadians(0)))


                                .waitSeconds(1)


                                .lineToSplineHeading(new Pose2d(-70, -40, Math.toRadians(90)))

                                /*.lineToSplineHeading(new Pose2d(-47, -40, Math.toRadians(0)),
                                        SampleMecanumDrive.getVelocityConstraint(30, DriveConstants.MAX_ANG_VEL,
                                                DriveConstants.TRACK_WIDTH), SampleMecanumDrive.getAccelerationConstraint((DriveConstants.MAX_ACCEL))

                                )*/

                                /* .lineToSplineHeading(new Pose2d(-44, -70, Math.toRadians(0)),
                                         SampleMecanumDrive.getVelocityConstraint(30, DriveConstants.MAX_ANG_VEL,
                                                 DriveConstants.TRACK_WIDTH), SampleMecanumDrive.getAccelerationConstraint((DriveConstants.MAX_ACCEL))

                                 )
             */

                                .lineToSplineHeading(new Pose2d(-45, -70, Math.toRadians(0)))

                                .lineToSplineHeading(new Pose2d(-45, -59, Math.toRadians(0)))
                                .waitSeconds(1)

                                .lineToSplineHeading(new Pose2d(14, -59, Math.toRadians(0)))




                                .lineToSplineHeading(new Pose2d(35, -29, Math.toRadians(0)))


                                .lineToSplineHeading(new Pose2d(45, -29, Math.toRadians(0)))

                                .waitSeconds(1)



                                .waitSeconds(1)

                                .setReversed(true)




                                .splineTo(new Vector2d(45, -60), Math.toRadians(-90))



                                .waitSeconds(5)


                                .build()
                );

        meepMeep.setBackground(MeepMeep.Background.FIELD_CENTERSTAGE_JUICE_LIGHT)
                .setDarkMode(false)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}