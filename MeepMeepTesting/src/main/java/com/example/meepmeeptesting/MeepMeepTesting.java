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


                                .addTemporalMarker(() -> {
                                    /*finger1.setPosition(sliderMachineState.stabFinger1Tight);
                                    finger2.setPosition(sliderMachineState.stabFinger2Tight);
*/
                                })
                                .addTemporalMarker(.9,() -> {

                                    /*moveByEncoder.powerSlider(slides, sliderMachineState.LOWpos);
                                    arm1.setPosition(sliderMachineState.armScore);
                                    arm2.setPosition(sliderMachineState.armScore);
                                    finger1.setPosition(sliderMachineState.stabFinger1Tight);
                                    finger2.setPosition(sliderMachineState.stabFinger2Tight);
                                    wrist.setPosition(sliderMachineState.wristScore);*/
                                })

                                .splineTo(new Vector2d(43, -28), Math.toRadians(0))
                                .waitSeconds(.01)
                                .splineTo(new Vector2d(51, -28), Math.toRadians(0))/*,
                                        SampleMecanumDrive.getVelocityConstraint(13, DriveConstants.MAX_ANG_VEL,
                                                DriveConstants.TRACK_WIDTH), SampleMecanumDrive.getAccelerationConstraint((DriveConstants.MAX_ACCEL)))
*/

                                //sensor function??

                                .waitSeconds(.1)
                                .addTemporalMarker(() -> {

                                    /*finger1.setPosition(sliderMachineState.Finger1Loose);
                                    finger2.setPosition(sliderMachineState.Finger2Loose);*/
                                    //release pixel
                                })

                                .setReversed(true)
                                .splineTo(new Vector2d(13, -33), Math.toRadians(180))

                                .addTemporalMarker(2.5,() -> {
                                    /*intakeLeft.setPosition(0);
                                    intakeRight.setPosition(0);*/

                                })

                                .addTemporalMarker(() -> {
                                   /* intake.setPower(-.4);*///svore purple
                                })

                                .lineToSplineHeading(new Pose2d(10, -33, Math.toRadians(0)))

                                //retract lift
                                .addTemporalMarker(() -> {
                                    /*moveByEncoder.powerSlider(slides, 0);
                                    arm1.setPosition(sliderMachineState.armThreaten);
                                    arm2.setPosition(sliderMachineState.armThreaten);
                                    finger1.setPosition(sliderMachineState.Finger1Loose);
                                    finger2.setPosition(sliderMachineState.Finger2Loose);
                                    wrist.setPosition(sliderMachineState.wristThreaten);*/

                                })

                                .addTemporalMarker(5,() -> {
//turn intake off after purple
                                    /*intake.setPower(0);*/
                                })

                                .setReversed(false)

                                .lineToSplineHeading(new Pose2d(13, -33, Math.toRadians(0)))
                                .splineTo(new Vector2d(30, -9), Math.toRadians(0))

//trying to score stack

                                //retract lift
                                .addTemporalMarker(() -> {

//prepare scoring to pass under gate
                                    /*moveByEncoder.powerSlider(slides, 0);
                                    arm1.setPosition(sliderMachineState.armThreaten);
                                    arm2.setPosition(sliderMachineState.armThreaten);
                                    finger1.setPosition(sliderMachineState.Finger1Loose);
                                    finger2.setPosition(sliderMachineState.Finger2Loose);
                                    wrist.setPosition(sliderMachineState.wristThreaten);
*/
                                })

                                .setReversed(true)
                                .splineTo(new Vector2d(20,-9), Math.toRadians(180))

                                //.lineToSplineHeading(new Pose2d(45, -5, Math.toRadians(0)))

                                .lineToSplineHeading(new Pose2d(-56, -9, Math.toRadians(0)))

                                .waitSeconds(.01)
                                .lineToSplineHeading(new Pose2d(-62, -11, Math.toRadians(0)))/*,
                                        SampleMecanumDrive.getVelocityConstraint(10, DriveConstants.MAX_ANG_VEL,
                                                DriveConstants.TRACK_WIDTH), SampleMecanumDrive.getAccelerationConstraint((DriveConstants.MAX_ACCEL)))
*/

                                //intake
                                .addTemporalMarker(() -> {

                                /*    intakeLeft.setPosition(.8);
                                    intakeRight.setPosition(.8);*/

//turn power on for intake
                                })

                                .lineToSplineHeading(new Pose2d(-55, -11, Math.toRadians(0)))

//NOW, we can turn on the intake!

                                .addTemporalMarker(() -> {
/*
                                    //create height for pixel 4
                                    intakeLeft.setPosition(.35);
                                    intakeRight.setPosition(.35);
                                    intake.setPower(1);*/

                                })

                                .waitSeconds(1)

                                //shifting over
                                .lineToSplineHeading(new Pose2d(-58.5, -11, Math.toRadians(0)))/*,
                                        SampleMecanumDrive.getVelocityConstraint(10, DriveConstants.MAX_ANG_VEL,
                                                DriveConstants.TRACK_WIDTH), SampleMecanumDrive.getAccelerationConstraint((DriveConstants.MAX_ACCEL)))
*/




/*
                    .waitSeconds(1)
                    .addTemporalMarker(() -> {
                        //create height for pixel 4
                        intakeLeft.setPosition(0);
                        intakeRight.setPosition(0);
                        intake.setPower(0);
                    })

//raise for second pixel and turn off power


                    //CYCLE 2!!


                    .waitSeconds(1)
                    .lineToSplineHeading(new Pose2d(-62, -9, Math.toRadians(0)),
                            SampleMecanumDrive.getVelocityConstraint(10, DriveConstants.MAX_ANG_VEL,
                                    DriveConstants.TRACK_WIDTH), SampleMecanumDrive.getAccelerationConstraint((DriveConstants.MAX_ACCEL)))

                    //intake
                    .addTemporalMarker(() -> {

                        intakeLeft.setPosition(.25);
                        intakeRight.setPosition(.25);


//turn power on for intake
                    })

                    .lineToSplineHeading(new Pose2d(-55, -9, Math.toRadians(0)))
                    .addTemporalMarker(() -> {

                        intakeLeft.setPosition(.35);
                        intakeRight.setPosition(.35);
                        intake.setPower(1);

                    })

                    .waitSeconds(1)


                    //shifting over
                    .lineToSplineHeading(new Pose2d(-60, -9, Math.toRadians(0)),
                            SampleMecanumDrive.getVelocityConstraint(10, DriveConstants.MAX_ANG_VEL,
                                    DriveConstants.TRACK_WIDTH), SampleMecanumDrive.getAccelerationConstraint((DriveConstants.MAX_ACCEL)))


                    //end of second

*/
                                .setReversed(false)
                                .splineTo(new Vector2d(-50, -9), Math.toRadians(0))//leaving the stack
                                //line to?

                                .addTemporalMarker(() -> {
                                    /*intakeLeft.setPosition(.35);
                                    intakeRight.setPosition(.35);*/
                                    //to pixk up pixels stuck in intake
                                })
                                .lineToSplineHeading(new Pose2d(30, -9, Math.toRadians(0)))//go thru gate

                                //raise lift


                                .addTemporalMarker(14,() -> {

                                    /*moveByEncoder.powerSlider(slides, 400);

                                    wrist.setPosition(sliderMachineState.wristStab);
                                    arm1.setPosition(sliderMachineState.armStab);
                                    arm2.setPosition(sliderMachineState.armStab);

*/
                                })
                                .addTemporalMarker(15,() -> {

                                    /*moveByEncoder.powerSlider(slides, -30);
                                    arm1.setPosition(sliderMachineState.armStab);
                                    arm2.setPosition(sliderMachineState.armStab);

                                    intake.setPower(-1);
*/
                                })
                                .addTemporalMarker(16,() -> {

                                    /*finger1.setPosition(sliderMachineState.stabFinger1Tight);
                                    finger2.setPosition(sliderMachineState.stabFinger2Tight);
*/
                                })

                                .setReversed(false)
                                .splineTo(new Vector2d(45,-35), Math.toRadians(0))

                                .addTemporalMarker(() -> {

                                    /*moveByEncoder.powerSlider(slides, sliderMachineState.MEDIUMpos);
                                    arm1.setPosition(sliderMachineState.armScore);
                                    arm2.setPosition(sliderMachineState.armScore);
                                    wrist.setPosition(sliderMachineState.wristScore);
*/

                                })

                                //go slow

                                .lineToSplineHeading(new Pose2d(49, -30, Math.toRadians(0)))/*,
                                        SampleMecanumDrive.getVelocityConstraint(15, DriveConstants.MAX_ANG_VEL,
                                                DriveConstants.TRACK_WIDTH), SampleMecanumDrive.getAccelerationConstraint((DriveConstants.MAX_ACCEL)))
*/
                                .waitSeconds(1)

                                .addTemporalMarker(() -> {

                                   /* finger1.setPosition(sliderMachineState.Finger1Loose);
                                    finger2.setPosition(sliderMachineState.Finger2Loose);
*/
                                    //release pixel
                                })

                                //end  of the extra

                                .waitSeconds(1)

                                .lineToSplineHeading(new Pose2d(45, -35, Math.toRadians(0)))//back off

                                .build()


                );

        meepMeep.setBackground(MeepMeep.Background.FIELD_CENTERSTAGE_JUICE_LIGHT)
                .setDarkMode(false)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}