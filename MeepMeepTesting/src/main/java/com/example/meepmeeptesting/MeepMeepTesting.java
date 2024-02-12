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




//begin stack


                                .lineToSplineHeading(new Pose2d(45, -34, Math.toRadians(0)))
                                .waitSeconds(.5)

                                //retract lift
                               .addTemporalMarker(() -> {


                                    /*moveByEncoder.powerSlider(slides, sliderMachineState.THREATENINGpos);
                                    arm1.setPosition(sliderMachineState.armThreaten);
                                    arm2.setPosition(sliderMachineState.armThreaten);
                                    finger1.setPosition(sliderMachineState.Finger1Loose);
                                    finger2.setPosition(sliderMachineState.Finger2Loose);
                                    wrist.setPosition(sliderMachineState.wristThreaten);
*/
                                })


                                .setReversed(true)
                                .splineTo(new Vector2d(25,-8), Math.toRadians(180))

                                //.lineToSplineHeading(new Pose2d(45, -5, Math.toRadians(0)))
                                .waitSeconds(.5)
                                .lineToSplineHeading(new Pose2d(-55, -8, Math.toRadians(0)))

                                .lineToSplineHeading(new Pose2d(-59, -10, Math.toRadians(0)))

                                //intake
                                .addTemporalMarker(15,() -> {


                                    //intake.setPower(-.75);

                                })

                                .waitSeconds(1)

                                //backup before the second one
                                .lineToSplineHeading(new Pose2d(-56, -10, Math.toRadians(0)))

                                .addTemporalMarker(17,() -> {
                                    /*intakeLeft.setPosition(.2);
                                    intakeRight.setPosition(.2);*/
                                })
                                .lineToSplineHeading(new Pose2d(-59, -10, Math.toRadians(0)))

                                .waitSeconds(1)

                                .addTemporalMarker(17,() -> {
                                    /*intakeLeft.setPosition(.35);
                                    intakeRight.setPosition(.35);*/
                                })


                                .lineToSplineHeading(new Pose2d(-50, -8, Math.toRadians(0)))
                                .waitSeconds(1)





                                //stab

                                .lineToSplineHeading(new Pose2d(40, -8, Math.toRadians(0)))
                                .waitSeconds(1)
                                //raise lift

                                .addTemporalMarker(20,() -> {

                                    //intake.setPower(0);

                                })

                                .addTemporalMarker(23,() -> {

                                    /*

                                    moveByEncoder.powerSlider(slides, 800);
                                    wrist.setPosition(wristStab);
                                    arm1.setPosition(armStab);
                                    arm2.setPosition(armStab);*/


                                })
                                .addTemporalMarker(24,() -> {

                                    /*

                                    moveByEncoder.powerSlider(slides, 0);


                                     */


                                })
                                .addTemporalMarker(25,() -> {

                                    /*

                                    stabberLeft.setPosition(slidePosition.stabFinger1Tight);
                                    stabberRight.setPosition(slidePosition.stabFinger2Tight);


                                     */
                                })




                                .setReversed(false)
                                .splineTo(new Vector2d(45,-30), Math.toRadians(0))

                                .waitSeconds(1)

                                //go slow
                                .lineToSplineHeading(new Pose2d(52, -30, Math.toRadians(0)))//insert custom y here
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