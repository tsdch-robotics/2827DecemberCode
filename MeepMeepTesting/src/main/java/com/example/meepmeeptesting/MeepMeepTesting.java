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

                                .addTemporalMarker(() -> {
                                    /*finger1.setPosition(sliderMachineState.stabFinger1Tight);
                                    finger2.setPosition(sliderMachineState.stabFinger2Tight);*/

                                })
                                .addTemporalMarker(.2,() -> {

                                    /*moveByEncoder.powerSlider(slides, sliderMachineState.LOWpos);
                                    arm1.setPosition(sliderMachineState.armScore);
                                    arm2.setPosition(sliderMachineState.armScore);
                                    finger1.setPosition(sliderMachineState.stabFinger1Tight);
                                    finger2.setPosition(sliderMachineState.stabFinger2Tight);
                                    wrist.setPosition(sliderMachineState.wristScore);*/
                                })


                                .splineTo(new Vector2d(45, -29.1), Math.toRadians(0))
                                .splineTo(new Vector2d(51, -29.2), Math.toRadians(0))//slow?
                                .waitSeconds(.1)
                                .addTemporalMarker(() -> {

                                    //finger1.setPosition(sliderMachineState.Finger1Loose);
                                    //finger2.setPosition(sliderMachineState.Finger2Loose);
                                    //release pixel
                                })

                                .setReversed(true)
                                .splineTo(new Vector2d(11, -33), Math.toRadians(180))
                                .addTemporalMarker(() -> {
                                    //intakeLeft.setPosition(.11);
                                    //intakeRight.setPosition(.11);*//*
                                    //intake.setPower(.4
                                })
                                //retract lift
                                .addTemporalMarker(() -> {
                                    /*moveByEncoder.powerSlider(slides, sliderMachineState.THREATENINGpos);
                                    arm1.setPosition(sliderMachineState.armThreaten);
                                    arm2.setPosition(sliderMachineState.armThreaten);
                                    finger1.setPosition(sliderMachineState.Finger1Loose);
                                    finger2.setPosition(sliderMachineState.Finger2Loose);
                                    wrist.setPosition(sliderMachineState.wristThreaten);*/

                                })

                                .addTemporalMarker(5,() -> {

                                    //intake.setPower(0
                                })

                                .setReversed(false)
                                .splineTo(new Vector2d(30, -9), Math.toRadians(0))

//trying to score stack

                                //retract lift
                                .addTemporalMarker(() -> {


                                    /*moveByEncoder.powerSlider(slides, sliderMachineState.THREATENINGpos);
                                    arm1.setPosition(sliderMachineState.armThreaten);
                                    arm2.setPosition(sliderMachineState.armThreaten);
                                    finger1.setPosition(sliderMachineState.Finger1Loose);
                                    finger2.setPosition(sliderMachineState.Finger2Loose);
                                    wrist.setPosition(sliderMachineState.wristThreaten);*/

                                })


                                .setReversed(true)
                                ///.splineTo(new Vector2d(20,-8), Math.toRadians(180))
                                //.lineToSplineHeading(new Pose2d(45, -5, Math.toRadians(0)))

                                .lineToSplineHeading(new Pose2d(-55, -8, Math.toRadians(0)))

                                .lineToSplineHeading(new Pose2d(-60, -10, Math.toRadians(0)))

                                //intake
                                .addDisplacementMarker(180,() -> {


                                    /*intake.setPower(-.75);*/

                                })

                                //.waitSeconds(.5)

                                //backup before the second one
                                .lineToSplineHeading(new Pose2d(-56, -10, Math.toRadians(0)))

                                .addTemporalMarker(() -> {
                                    /*intakeLeft.setPosition(.2);
                                    intakeRight.setPosition(.2);*/
                                })
                                .waitSeconds(.5)

                                .lineToSplineHeading(new Pose2d(-60, -10, Math.toRadians(0)))




                                .lineToSplineHeading(new Pose2d(-50, -8, Math.toRadians(0)))//leaving the stack
                                .addTemporalMarker(() -> {
                                    /*intakeLeft.setPosition(.35);
                                    intakeRight.setPosition(.35);*/
                                })
                                .lineToSplineHeading(new Pose2d(40, -8, Math.toRadians(0)))//go thru gate

                                //raise lift

                                .addDisplacementMarker(350,() -> {

                                    /*intake.setPower(0);*/

                                })

                                .addDisplacementMarker(390,() -> {

                                   /* moveByEncoder.powerSlider(slides, 800);
                                    wrist.setPosition(sliderMachineState.wristStab);
                                    arm1.setPosition(sliderMachineState.armStab);
                                    arm2.setPosition(sliderMachineState.armStab);
*/

                                })
                                .addTemporalMarker(24,() -> {

/*


                                    moveByEncoder.powerSlider(slides, 0);



                                })
                                .addTemporalMarker(25,() -> {


/*

                                    finger1.setPosition(sliderMachineState.Finger1Loose);
                                    finger2.setPosition(sliderMachineState.Finger2Loose);

*/


                                })




                                .setReversed(false)
                                .splineTo(new Vector2d(45,-30), Math.toRadians(0))
                                .waitSeconds(.01)

                                //go slow
                                .lineToSplineHeading(new Pose2d(52, -35, Math.toRadians(0)))//insert custom y here
                                .waitSeconds(.01)


                                .addTemporalMarker(() -> {


                                   /* moveByEncoder.powerSlider(slides, sliderMachineState.LOWpos);
                                    arm1.setPosition(sliderMachineState.armScore);
                                    arm2.setPosition(sliderMachineState.armScore);
                                    finger1.setPosition(sliderMachineState.stabFinger1Tight);
                                    finger2.setPosition(sliderMachineState.stabFinger2Tight);
                                    wrist.setPosition(sliderMachineState.wristScore);

*/
                                })
                                .waitSeconds(1.5)
                                .addTemporalMarker(() -> {

                                    /*finger1.setPosition(sliderMachineState.Finger1Loose);
                                    finger2.setPosition(sliderMachineState.Finger2Loose);

                                    */

                                    //release pixel
                                })






                                //end  of the extra 1



//attempt 2


                                .setReversed(true)
                                .splineTo(new Vector2d(20,-9), Math.toRadians(0))

                                //retract lift
                                .addTemporalMarker(() -> {


                                    /*moveByEncoder.powerSlider(slides, sliderMachineState.THREATENINGpos);
                                    arm1.setPosition(sliderMachineState.armThreaten);
                                    arm2.setPosition(sliderMachineState.armThreaten);
                                    finger1.setPosition(sliderMachineState.Finger1Loose);
                                    finger2.setPosition(sliderMachineState.Finger2Loose);
                                    wrist.setPosition(sliderMachineState.wristThreaten);*/

                                })


                                .setReversed(true)
                                ///.splineTo(new Vector2d(20,-8), Math.toRadians(180))
                                //.lineToSplineHeading(new Pose2d(45, -5, Math.toRadians(0)))

                                .lineToSplineHeading(new Pose2d(-55, -8, Math.toRadians(0)))

                                .lineToSplineHeading(new Pose2d(-60, -10, Math.toRadians(0)))

                                //intake
                                .addDisplacementMarker(180,() -> {


                                    /*intake.setPower(-.75);*/

                                })

                                //.waitSeconds(.5)

                                //backup before the second one
                                .lineToSplineHeading(new Pose2d(-56, -10, Math.toRadians(0)))

                                .addTemporalMarker(() -> {
                                    /*intakeLeft.setPosition(.2);
                                    intakeRight.setPosition(.2);*/
                                })
                                .waitSeconds(.5)

                                .lineToSplineHeading(new Pose2d(-60, -10, Math.toRadians(0)))




                                .lineToSplineHeading(new Pose2d(-50, -8, Math.toRadians(0)))//leaving the stack
                                .addTemporalMarker(() -> {
                                    /*intakeLeft.setPosition(.35);
                                    intakeRight.setPosition(.35);*/
                                })
                                .lineToSplineHeading(new Pose2d(40, -8, Math.toRadians(0)))//go thru gate

                                //raise lift

                                .addDisplacementMarker(350,() -> {

                                    /*intake.setPower(0);*/

                                })

                                .addDisplacementMarker(390,() -> {

                                   /* moveByEncoder.powerSlider(slides, 800);
                                    wrist.setPosition(sliderMachineState.wristStab);
                                    arm1.setPosition(sliderMachineState.armStab);
                                    arm2.setPosition(sliderMachineState.armStab);
*/

                                })
                                .addTemporalMarker(24,() -> {

/*


                                    moveByEncoder.powerSlider(slides, 0);



                                })
                                .addTemporalMarker(25,() -> {


/*

                                    finger1.setPosition(sliderMachineState.Finger1Loose);
                                    finger2.setPosition(sliderMachineState.Finger2Loose);

*/


                                })




                                .setReversed(false)
                                .splineTo(new Vector2d(45,-30), Math.toRadians(0))
                                .waitSeconds(.01)

                                //go slow
                                .lineToSplineHeading(new Pose2d(52, -35, Math.toRadians(0)))//insert custom y here
                                .waitSeconds(.01)


                                .addTemporalMarker(() -> {


                                   /* moveByEncoder.powerSlider(slides, sliderMachineState.LOWpos);
                                    arm1.setPosition(sliderMachineState.armScore);
                                    arm2.setPosition(sliderMachineState.armScore);
                                    finger1.setPosition(sliderMachineState.stabFinger1Tight);
                                    finger2.setPosition(sliderMachineState.stabFinger2Tight);
                                    wrist.setPosition(sliderMachineState.wristScore);

*/
                                })
                                .waitSeconds(1.5)
                                .addTemporalMarker(() -> {

                                    /*finger1.setPosition(sliderMachineState.Finger1Loose);
                                    finger2.setPosition(sliderMachineState.Finger2Loose);

                                    */

                                    //release pixel
                                })






                                //end  of the extra 2



                                .build()


                );

        meepMeep.setBackground(MeepMeep.Background.FIELD_CENTERSTAGE_JUICE_LIGHT)
                .setDarkMode(false)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}