//trying to score stack
                    /*.lineToSplineHeading(new Pose2d(45, -34, Math.toRadians(0)))
                            .waitSeconds(.1)

                            //retract lift
                            .addTemporalMarker(() -> {


                            moveByEncoder.powerSlider(slides, sliderMachineState.THREATENINGpos);
                            arm1.setPosition(sliderMachineState.armThreaten);
                            arm2.setPosition(sliderMachineState.armThreaten);
                            finger1.setPosition(sliderMachineState.Finger1Loose);
                            finger2.setPosition(sliderMachineState.Finger2Loose);
                            wrist.setPosition(sliderMachineState.wristThreaten);

                            })

                            .lineToSplineHeading(new Pose2d(45, -10, Math.toRadians(0)))
                            .waitSeconds(.1)
                            .lineToSplineHeading(new Pose2d(-50, -10, Math.toRadians(0)))
                            .waitSeconds(.1)


                            //turn on slow
                            .lineToSplineHeading(new Pose2d(-55, -12, Math.toRadians(0)))
                            .waitSeconds(2)

                            //intake
                            .addTemporalMarker(() -> {

                            intake.setPower(.5);

                            })

                            .waitSeconds(2)
                            .lineToSplineHeading(new Pose2d(-50, -10, Math.toRadians(0)))
                            .waitSeconds(.1)
                            //stab

                            .lineToSplineHeading(new Pose2d(45, -10, Math.toRadians(0)))
                            .waitSeconds(.1)
                            //raise lift


                            .lineToSplineHeading(new Pose2d(45, -30, Math.toRadians(0)))//insert custom y here
                            .waitSeconds(.1)

                            //go slow
                            .lineToSplineHeading(new Pose2d(52, -30, Math.toRadians(0)))//insert custom y here
                            .waitSeconds(.1)

*/