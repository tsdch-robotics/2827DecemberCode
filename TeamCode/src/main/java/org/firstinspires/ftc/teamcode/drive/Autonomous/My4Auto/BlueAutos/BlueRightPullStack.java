package org.firstinspires.ftc.teamcode.drive.Autonomous.My4Auto.BlueAutos;


import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.drive.Drive;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.drive.Autonomous.Misc.PoseStorage;
import org.firstinspires.ftc.teamcode.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.drive.TeleOp.DavidsFUNctions.moveWithBasicEncoder;
import org.firstinspires.ftc.teamcode.drive.TeleOp.DavidsFUNctions.sliderMachineState;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvPipeline;
import org.openftc.easyopencv.OpenCvWebcam;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.hardware.bosch.BNO055IMU;

@Config
@Autonomous(group = "drive", preselectTeleOp = "Run this TeleOp!")
public class BlueRightPullStack extends LinearOpMode {

    private double autonomousTimeOffset = 0.0;

    private Servo intakeLeft;
    private Servo intakeRight;
    private DcMotor intake;



    public ElapsedTime slidesTime = new ElapsedTime();

    OpenCvWebcam webcam1 = null;

    ElapsedTime poseTransferTime = new ElapsedTime();

    moveWithBasicEncoder moveByEncoder = new moveWithBasicEncoder();

    int frameCount = 0;
    int zone = 0;
    int totalLeft;
    int totalRight;

    public boolean greaterThanTargetPercentBluePixels1 = false;
    public boolean greaterThanTargetPercentBluePixels2 = false;

    ColorAnalysisPipeline colorAnalysisPipeline;

    private double targetPixPercent1 = .2;
    private double targetPixPercent2 = .2;

    private DcMotor slides;

    private Servo arm1;
    private Servo arm2;
    private Servo wrist;
    private Servo finger1;
    private Servo finger2;
    private Servo flicker;


    class ColorAnalysisPipeline extends OpenCvPipeline {

        Mat roi1 = new Mat();
        Mat roi2 = new Mat();

        public boolean firstTime = true;

        Mat blueMask = new Mat();

        Mat inputMat = new Mat();
        Mat output = new Mat();




        Scalar redLower = new Scalar(0, 50, 50);
        Scalar redUpper = new Scalar(20, 255, 255);
        Scalar yellowLower = new Scalar(20, 50, 50);
        Scalar yellowUpper = new Scalar(40, 255, 255);
        Scalar blueLower = new Scalar(90, 50, 50);
        Scalar blueUpper = new Scalar(130, 255, 255);

        private Rect rect1 = new Rect(50, 150, 300, 150);
        private Rect rect2 = new Rect(440, 120, 190, 180);

        private int bluePixels1;
        private int bluePixels2;

        public int getBluePixels1() {
            return bluePixels1;
        }

        public int getBluePixels2() {
            return bluePixels2;
        }

        public Rect getRect1() {
            return rect1;
        }

        public Rect getRect2() {
            return rect2;
        }

        @Override
        public Mat processFrame(Mat input) {


            output.release();
            inputMat.release();


            input.copyTo(this.inputMat);

            this.inputMat.copyTo(this.output);

            roi1 = this.inputMat.submat(rect1);
            roi2 = this.inputMat.submat(rect2);



            bluePixels1 = countBluePixels(roi1);
            bluePixels2 = countBluePixels(roi2);

            telemetry.addData("ProcessFrame Called", true);
            telemetry.addData("Blue Pixels in Rectangle 1", bluePixels1);
            telemetry.addData("Blue Pixels in Rectangle 2", bluePixels2);
            telemetry.addData("Rectangle1 Area", rect1.area());
            telemetry.addData("Rectangle2 Area", rect2.area());

            Imgproc.rectangle(this.output, rect1, new Scalar(255, 0, 0), 2);
            Imgproc.rectangle(this.output, rect2, new Scalar(255, 0, 0), 2);


            roi1.release();
            roi2.release();

            return this.output;
        }

        private int countBluePixels(Mat image) {
            Imgproc.cvtColor(image, image, Imgproc.COLOR_RGB2HSV);

            //Mat blueMask = new Mat();
            Core.inRange(image, blueLower, blueUpper, blueMask);

            return Core.countNonZero(blueMask);
        }
    }

    @Override
    public void runOpMode() throws InterruptedException {

        poseTransferTime.startTime();
        poseTransferTime.reset();

        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        Pose2d startPose = new Pose2d(-38, 60, Math.toRadians(-90));
        drive.setPoseEstimate(startPose);

        drive.setPoseEstimate(startPose);



        WebcamName webcamName = hardwareMap.get(WebcamName.class, "Webcam 1");
        int cameraMonitorViewId = hardwareMap.appContext.getResources()
                .getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam1 = OpenCvCameraFactory.getInstance().createWebcam(webcamName, cameraMonitorViewId);


        intakeLeft = hardwareMap.servo.get("intakeLeft");
        intakeRight = hardwareMap.servo.get("intakeRight");
        intake = hardwareMap.dcMotor.get("intake");
        intake.setDirection(DcMotorSimple.Direction.REVERSE);

        slides = hardwareMap.dcMotor.get("slides");
        slides.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        slides.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        arm1 = hardwareMap.servo.get("arm1");
        arm2 = hardwareMap.servo.get("arm2");
        wrist = hardwareMap.servo.get("wrist");
        finger1 = hardwareMap.servo.get("finger1");
        finger2 = hardwareMap.servo.get("finger2");
        flicker = hardwareMap.servo.get("flicker");

        arm2.setDirection(Servo.Direction.REVERSE);



        colorAnalysisPipeline = new ColorAnalysisPipeline();
        webcam1.setPipeline(colorAnalysisPipeline);

        webcam1.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            public void onOpened() {
                webcam1.startStreaming(640, 480, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode) {

            }
        });


        telemetry.addLine("Waiting to start");

        telemetry.addData("BluePixels1", colorAnalysisPipeline.getBluePixels1());
        telemetry.addData("BluePixels1", colorAnalysisPipeline.getBluePixels2());
        telemetry.update();


        waitForStart();

        poseTransferTime.startTime();
        poseTransferTime.reset();

        telemetry.addLine("started");
        telemetry.update();

        for (int i = 0; i < 500; i++) {
            telemetry.addLine("Measuring Camera stream");
            telemetry.addData("totalLeft", totalLeft);
            telemetry.addData("totalLeft", totalRight);
            telemetry.update();

            // Process frames and accumulate color values
            totalLeft += colorAnalysisPipeline.getBluePixels1();
            totalRight += colorAnalysisPipeline.getBluePixels2();

            frameCount = frameCount + 1;
            sleep(0);
        }

        webcam1.stopStreaming();//may want to remove

        double averageLeft = totalLeft / frameCount;
        double averageRight = totalRight / frameCount;

        if (averageLeft / colorAnalysisPipeline.getRect1().area() > targetPixPercent1) {
            greaterThanTargetPercentBluePixels1 = true;
        } else if (averageRight / colorAnalysisPipeline.getRect2().area() > targetPixPercent2) {
            greaterThanTargetPercentBluePixels2 = true;
        } else {
            greaterThanTargetPercentBluePixels1 = false;
            greaterThanTargetPercentBluePixels2 = false;
        }

        if (averageLeft > averageRight && greaterThanTargetPercentBluePixels1) {
            telemetry.addLine("Middle");
            zone = 2;
        } else if (averageLeft < averageRight && greaterThanTargetPercentBluePixels2) {
            telemetry.addLine("Right");
            zone = 3;
        } else {
            telemetry.addLine("Left");
            zone = 1;
        }

        telemetry.addLine("done computing");
        telemetry.addData("left", averageLeft);
        telemetry.addData("leftTotal", totalLeft);
        telemetry.addData("right", averageRight);
        telemetry.update();


        if (zone == 2) {
            telemetry.addLine("running zone 2 auto!");
            telemetry.update();

            //purple: .lineToSplineHeading(new Pose2d(-38.0, 30.6, Math.toRadians(-90)))
            //score: .lineToSplineHeading(new Pose2d(52, 33, Math.toRadians(-2))

            TrajectorySequence trajectory1 = drive.trajectorySequenceBuilder(startPose)



                    .waitSeconds(autonomousTimeOffset)

                    .addTemporalMarker(() -> {
                        flicker.setPosition(.5);
                        finger1.setPosition(sliderMachineState.stabFinger1Tight);
                        finger2.setPosition(sliderMachineState.stabFinger2Tight);
                    })


                    .splineTo(new Vector2d(-38.2, 30.7), Math.toRadians(0))

                    .addTemporalMarker(() -> {
                        flicker.setPosition(.8);

                        moveByEncoder.powerSlider(slides, 500);
                        arm1.setPosition(sliderMachineState.armThreaten);
                        arm2.setPosition(sliderMachineState.armThreaten);
                        finger1.setPosition(sliderMachineState.Finger2Loose);
                        finger2.setPosition(sliderMachineState.stabFinger2Tight);
                        wrist.setPosition(sliderMachineState.wristThreaten);
                    })

                    .waitSeconds(.1)

                    .setReversed(true)
                    .splineTo(new Vector2d(-46, 40), Math.toRadians(180))


                    //put up the intake
                    .addTemporalMarker(() -> {

                        //create height for pixel 4
                        intakeLeft.setPosition(0);
                        intakeRight.setPosition(0);


                    })

                    .lineToSplineHeading(new Pose2d(-62, 40, Math.toRadians(0)))

                    .addTemporalMarker(() -> {

                        //create height for pixel 4
                        intakeLeft.setPosition(.05);
                        intakeRight.setPosition(.05);


                    })

                    .lineToSplineHeading(new Pose2d(-55, 40, Math.toRadians(0)))


                    .addTemporalMarker(() -> {

                        //create height for pixel 4
                        intakeLeft.setPosition(.35);
                        intakeRight.setPosition(.35);
                        intake.setPower(1);

                    })


                    //turn on power, then...
                    .lineToSplineHeading(new Pose2d(-57, 40, Math.toRadians(0)))

                    //wait a second, then stab while holding the yellow

                    .addTemporalMarker(8 + autonomousTimeOffset,() -> {

                        moveByEncoder.powerSlider(slides, 500);
                        arm1.setPosition(sliderMachineState.armStab);
                        arm2.setPosition(sliderMachineState.armStab);
                        finger1.setPosition(sliderMachineState.Finger2Loose);
                        finger2.setPosition(sliderMachineState.stabFinger2Tight);
                        wrist.setPosition(sliderMachineState.wristStab);
                    })

                    .addTemporalMarker(7 + autonomousTimeOffset,() -> {

                        moveByEncoder.powerSlider(slides, 0);
                        arm1.setPosition(sliderMachineState.armStab);
                        arm2.setPosition(sliderMachineState.armStab);
                        finger1.setPosition(sliderMachineState.Finger2Loose);
                        finger2.setPosition(sliderMachineState.stabFinger2Tight);
                        wrist.setPosition(sliderMachineState.wristStab);
                    })

                    .addTemporalMarker(9 + autonomousTimeOffset,() -> {

                        finger1.setPosition(sliderMachineState.stabFinger1Tight);
                        finger2.setPosition(sliderMachineState.stabFinger2Tight);

                    })





                    //continue to the rest of the code
                    .lineToSplineHeading(new Pose2d(-37, 58.7, Math.toRadians(0)))



                    .lineToSplineHeading(new Pose2d(13.1, 58.7, Math.toRadians(0)))

                    .waitSeconds(.1)

                    .addTemporalMarker(() -> {


                        moveByEncoder.powerSlider(slides, sliderMachineState.LOWpos);
                        arm1.setPosition(sliderMachineState.armScore);
                        arm2.setPosition(sliderMachineState.armScore);
                        finger1.setPosition(sliderMachineState.stabFinger1Tight);
                        finger2.setPosition(sliderMachineState.stabFinger2Tight);
                        wrist.setPosition(sliderMachineState.wristScore);

                    })


                    .waitSeconds(.1)

                    .splineTo(new Vector2d(44.5, 35), Math.toRadians(0))
                    .splineTo(new Vector2d(52, 35), Math.toRadians(-5),
                            SampleMecanumDrive.getVelocityConstraint(15, DriveConstants.MAX_ANG_VEL,
                                    DriveConstants.TRACK_WIDTH), SampleMecanumDrive.getAccelerationConstraint((DriveConstants.MAX_ACCEL)))
//slow to board


                    .waitSeconds(.5)

                    .addTemporalMarker(() -> {

                        finger1.setPosition(sliderMachineState.Finger1Loose);
                        finger2.setPosition(sliderMachineState.Finger2Loose);

                    })

                    .waitSeconds(.5)




                    .lineToSplineHeading(new Pose2d(47, 40, Math.toRadians(0)))
                    .waitSeconds(.1)

                    .setReversed(true)
                    .splineTo(new Vector2d(46.5, 55), Math.toRadians(90))//slo
                    .build();



            drive.followTrajectorySequence(trajectory1);



        } else if (zone == 3) {
            telemetry.addLine("running zone 3 auto!");
            telemetry.update();

            //purple: .lineToSplineHeading(new Pose2d(-38.67, 32.4, Math.toRadians(180)))
            //score: .lineToSplineHeading(new Pose2d(53.5, 27, Math.toRadians(0)),
            TrajectorySequence trajectory1 = drive.trajectorySequenceBuilder(startPose)

                    .waitSeconds(autonomousTimeOffset)

                    .addTemporalMarker(() -> {
                        flicker.setPosition(.5);
                        finger1.setPosition(sliderMachineState.stabFinger1Tight);
                        finger2.setPosition(sliderMachineState.stabFinger2Tight);
                    })

                    .lineToSplineHeading(new Pose2d(-40.9, 51.8, Math.toRadians(90)))

                    .splineTo(new Vector2d(-32.4, -34.3), Math.toRadians(0))
                    .lineToSplineHeading(new Pose2d(-32.41, 34.31, Math.toRadians(0)))//spike mark

                    .waitSeconds(.1)

                    .addTemporalMarker(() -> {
                        flicker.setPosition(.8);

                        moveByEncoder.powerSlider(slides, 500);
                        arm1.setPosition(sliderMachineState.armThreaten);
                        arm2.setPosition(sliderMachineState.armThreaten);
                        finger1.setPosition(sliderMachineState.Finger2Loose);
                        finger2.setPosition(sliderMachineState.stabFinger2Tight);
                        wrist.setPosition(sliderMachineState.wristThreaten);
                    })

                    .waitSeconds(.1)

                    .lineToSplineHeading(new Pose2d(-38.5, 34.3, Math.toRadians(0)))
                    //take one pixel off stack



                    //put up the intake


                    .addTemporalMarker(() -> {

                        //create height for pixel 4
                        intakeLeft.setPosition(0);
                        intakeRight.setPosition(0);


                    })

                    .lineToSplineHeading(new Pose2d(-62, 40, Math.toRadians(0)))


                    .addTemporalMarker(() -> {

                        //create height for pixel 4
                        intakeLeft.setPosition(.05);
                        intakeRight.setPosition(.05);


                    })

                    .lineToSplineHeading(new Pose2d(-55, 40, Math.toRadians(0)))


                    .addTemporalMarker(() -> {

                        //create height for pixel 4
                        intakeLeft.setPosition(.35);
                        intakeRight.setPosition(.35);
                        intake.setPower(1);

                    })


                    //turn on power, then...
                    .lineToSplineHeading(new Pose2d(-57, 40, Math.toRadians(0)))

                    //wait a second, then stab while holding the yellow

                    .addTemporalMarker(8 + autonomousTimeOffset,() -> {

                        moveByEncoder.powerSlider(slides, 500);
                        arm1.setPosition(sliderMachineState.armStab);
                        arm2.setPosition(sliderMachineState.armStab);
                        finger1.setPosition(sliderMachineState.Finger2Loose);
                        finger2.setPosition(sliderMachineState.stabFinger2Tight);
                        wrist.setPosition(sliderMachineState.wristStab);
                    })

                    .addTemporalMarker(7 + autonomousTimeOffset,() -> {

                        moveByEncoder.powerSlider(slides, 0);
                        arm1.setPosition(sliderMachineState.armStab);
                        arm2.setPosition(sliderMachineState.armStab);
                        finger1.setPosition(sliderMachineState.Finger2Loose);
                        finger2.setPosition(sliderMachineState.stabFinger2Tight);
                        wrist.setPosition(sliderMachineState.wristStab);
                    })

                    .addTemporalMarker(9 + autonomousTimeOffset,() -> {

                        finger1.setPosition(sliderMachineState.stabFinger1Tight);
                        finger2.setPosition(sliderMachineState.stabFinger2Tight);

                    })





                    //continue to the rest of the code
                    .lineToSplineHeading(new Pose2d(-37, 58.7, Math.toRadians(0)))



                    .lineToSplineHeading(new Pose2d(13.1, 58.7, Math.toRadians(0)))

                    .waitSeconds(.1)

                    .addTemporalMarker(() -> {


                        moveByEncoder.powerSlider(slides, sliderMachineState.LOWpos);
                        arm1.setPosition(sliderMachineState.armScore);
                        arm2.setPosition(sliderMachineState.armScore);
                        finger1.setPosition(sliderMachineState.stabFinger1Tight);
                        finger2.setPosition(sliderMachineState.stabFinger2Tight);
                        wrist.setPosition(sliderMachineState.wristScore);

                    })


                    .waitSeconds(.1)

                    .splineTo(new Vector2d(44.5, 39.25), Math.toRadians(0))


                    .splineTo(new Vector2d(51, 39.25), Math.toRadians(0),
                            SampleMecanumDrive.getVelocityConstraint(15, DriveConstants.MAX_ANG_VEL,
                                    DriveConstants.TRACK_WIDTH), SampleMecanumDrive.getAccelerationConstraint((DriveConstants.MAX_ACCEL)))
//slow to board



                    .waitSeconds(.5)

                    .addTemporalMarker(() -> {

                        finger1.setPosition(sliderMachineState.Finger1Loose);
                        finger2.setPosition(sliderMachineState.Finger2Loose);

                    })

                    .waitSeconds(.5)

                    .lineToSplineHeading(new Pose2d(47, 40, Math.toRadians(0)))
                    .waitSeconds(.1)

                    .setReversed(true)
                    .splineTo(new Vector2d(46.5, 55), Math.toRadians(90))//slow


                    .build();



            drive.followTrajectorySequence(trajectory1);




        } else {
            telemetry.addLine("running zone 1 auto!");
            telemetry.update();
//Purple: .lineToSplineHeading(new Pose2d(-30.36, 34.45, Math.toRadians(0)))
            //score: .splineTo(new Vector2d(51.5, 39.1), Math.toRadians(-5),

            TrajectorySequence trajectory1 = drive.trajectorySequenceBuilder(startPose)


                    .waitSeconds(autonomousTimeOffset)

                    .addTemporalMarker(() -> {
                        flicker.setPosition(.5);
                        finger1.setPosition(sliderMachineState.stabFinger1Tight);
                        finger2.setPosition(sliderMachineState.stabFinger2Tight);
                    })


                    .splineTo(new Vector2d(-46, 36.7), Math.toRadians(0))

                    .addTemporalMarker(() -> {
                        flicker.setPosition(.8);

                        moveByEncoder.powerSlider(slides, 500);
                        arm1.setPosition(sliderMachineState.armThreaten);
                        arm2.setPosition(sliderMachineState.armThreaten);
                        finger1.setPosition(sliderMachineState.Finger2Loose);
                        finger2.setPosition(sliderMachineState.stabFinger2Tight);
                        wrist.setPosition(sliderMachineState.wristThreaten);
                    })

                    .waitSeconds(.1)

                    .setReversed(true)
                    .splineTo(new Vector2d(-46, 40), Math.toRadians(180))


                    //put up the intake
                    .addTemporalMarker(() -> {

                        //create height for pixel 4
                        intakeLeft.setPosition(0);
                        intakeRight.setPosition(0);


                    })

                    .lineToSplineHeading(new Pose2d(-62, 40, Math.toRadians(0)))

                    .addTemporalMarker(() -> {

                        //create height for pixel 4
                        intakeLeft.setPosition(.05);
                        intakeRight.setPosition(.05);


                    })

                    .lineToSplineHeading(new Pose2d(-55, 40, Math.toRadians(0)))


                    .addTemporalMarker(() -> {

                        //create height for pixel 4
                        intakeLeft.setPosition(.35);
                        intakeRight.setPosition(.35);
                        intake.setPower(1);

                    })


                    //turn on power, then...
                    .lineToSplineHeading(new Pose2d(-57, 40, Math.toRadians(0)))

                    //wait a second, then stab while holding the yellow

                    .addTemporalMarker(8 + autonomousTimeOffset,() -> {

                        moveByEncoder.powerSlider(slides, 500);
                        arm1.setPosition(sliderMachineState.armStab);
                        arm2.setPosition(sliderMachineState.armStab);
                        finger1.setPosition(sliderMachineState.Finger2Loose);
                        finger2.setPosition(sliderMachineState.stabFinger2Tight);
                        wrist.setPosition(sliderMachineState.wristStab);
                    })

                    .addTemporalMarker(7 + autonomousTimeOffset,() -> {

                        moveByEncoder.powerSlider(slides, 0);
                        arm1.setPosition(sliderMachineState.armStab);
                        arm2.setPosition(sliderMachineState.armStab);
                        finger1.setPosition(sliderMachineState.Finger2Loose);
                        finger2.setPosition(sliderMachineState.stabFinger2Tight);
                        wrist.setPosition(sliderMachineState.wristStab);
                    })

                    .addTemporalMarker(9 + autonomousTimeOffset,() -> {

                        finger1.setPosition(sliderMachineState.stabFinger1Tight);
                        finger2.setPosition(sliderMachineState.stabFinger2Tight);

                    })





                    //continue to the rest of the code
                    .lineToSplineHeading(new Pose2d(-37, -58.7, Math.toRadians(0)))



                    .lineToSplineHeading(new Pose2d(13.1, 58.7, Math.toRadians(0)))

                    .waitSeconds(.1)

                    .addTemporalMarker(() -> {


                        moveByEncoder.powerSlider(slides, sliderMachineState.LOWpos);
                        arm1.setPosition(sliderMachineState.armScore);
                        arm2.setPosition(sliderMachineState.armScore);
                        finger1.setPosition(sliderMachineState.stabFinger1Tight);
                        finger2.setPosition(sliderMachineState.stabFinger2Tight);
                        wrist.setPosition(sliderMachineState.wristScore);

                    })


                    .waitSeconds(.1)

                    .splineTo(new Vector2d(44.5, 30.1), Math.toRadians(0))
                    .splineTo(new Vector2d(52, 30.1), Math.toRadians(-5),
                            SampleMecanumDrive.getVelocityConstraint(15, DriveConstants.MAX_ANG_VEL,
                                    DriveConstants.TRACK_WIDTH), SampleMecanumDrive.getAccelerationConstraint((DriveConstants.MAX_ACCEL)))
//slow to board


                    .waitSeconds(.5)

                    .addTemporalMarker(() -> {

                        finger1.setPosition(sliderMachineState.Finger1Loose);
                        finger2.setPosition(sliderMachineState.Finger2Loose);

                    })

                    .waitSeconds(.5)


                    .lineToSplineHeading(new Pose2d(47, 40, Math.toRadians(0)))
                    .waitSeconds(.1)

                    .setReversed(true)
                    .splineTo(new Vector2d(46.5, 55), Math.toRadians(90))//slow


                    .build();


            drive.followTrajectorySequence(trajectory1);




        }
    }

}