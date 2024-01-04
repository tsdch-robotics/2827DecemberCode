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
                        drive.trajectorySequenceBuilder(new Pose2d(15, 60, Math.toRadians(270)))//-38, 60 for blue right



                                .splineTo(new Vector2d(14, 33), Math.toRadians(-90))//middle pos
                                .waitSeconds(.5)
                                //.addTemporalMarker(() -> executeSlides.magicalMacro(slides, arm1, arm2, wrist, finger1, finger2, sliderMachineState.slidePosition.LOW, slidesTime, true))

                                .lineToSplineHeading(new Pose2d(15, 45, Math.toRadians(-80)))
                                .waitSeconds(.25)




                                .splineTo(new Vector2d(47, 35), Math.toRadians(0))//to board

                                .waitSeconds(1)


                                .setReversed(true)//intaking side is the front now
                                .splineTo(new Vector2d(45, 58), Math.toRadians(90))


                                //  .splineTo(new Vector2d(28, 12), Math.toRadians(180))//180 because its backwards
                               // .splineTo(new Vector2d(-28, 12), Math.toRadians(180))
                               // .splineTo(new Vector2d(-55, 16), Math.toRadians(180))

                                //.setReversed(false)
                              //  .splineTo(new Vector2d(-28, 12), Math.toRadians(0))
                              //  .splineTo(new Vector2d(28, 12), Math.toRadians(0))//180 because its backwards
                              //  .splineTo(new Vector2d(47, 35), Math.toRadians(0))

//test





                                .build()
                );

        meepMeep.setBackground(MeepMeep.Background.FIELD_CENTERSTAGE_JUICE_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}