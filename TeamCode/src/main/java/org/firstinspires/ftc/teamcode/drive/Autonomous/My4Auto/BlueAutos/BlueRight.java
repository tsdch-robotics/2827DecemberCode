package org.firstinspires.ftc.teamcode.drive.Autonomous.My4Auto.BlueAutos;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.drive.Drive;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
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

@Config
@Autonomous(group = "drive", preselectTeleOp = "Run this TeleOp!")
public class BlueRight extends LinearOpMode {

    public ElapsedTime slidesTime = new ElapsedTime();
    sliderMachineState executeSlides = new sliderMachineState();

    OpenCvWebcam webcam1 = null;
    ElapsedTime elapsedTime = new ElapsedTime(); // Add ElapsedTime to track time

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



        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        Pose2d startPose = new Pose2d(-38, 60, Math.toRadians(-90));
        drive.setPoseEstimate(startPose);

        drive.setPoseEstimate(startPose);





        WebcamName webcamName = hardwareMap.get(WebcamName.class, "Webcam 1");
        int cameraMonitorViewId = hardwareMap.appContext.getResources()
                .getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam1 = OpenCvCameraFactory.getInstance().createWebcam(webcamName, cameraMonitorViewId);







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





            TrajectorySequence trajectory1 = drive.trajectorySequenceBuilder(startPose)


                    .addTemporalMarker(() -> {
                        flicker.setPosition(.5);
                        finger1.setPosition(sliderMachineState.stabFinger1Tight);
                        finger2.setPosition(sliderMachineState.stabFinger2Tight);
                    })

                    .lineToSplineHeading(new Pose2d(-38.0, 30.6, Math.toRadians(-90)))


                    .waitSeconds(.1)

                    .addTemporalMarker(() -> {flicker.setPosition(.8);} )


                    .waitSeconds(.1)


                    .lineToSplineHeading(new Pose2d(-42.5, 58.1, Math.toRadians(0)))//prepare for journey thru tus


                    .waitSeconds(.1)


                    .lineToSplineHeading(new Pose2d(17.4, 58.1, Math.toRadians(0)))


                    .addTemporalMarker(() -> {

                        moveByEncoder.powerSlider(slides, sliderMachineState.LOWpos + 200);
                        arm1.setPosition(sliderMachineState.armScore);
                        arm2.setPosition(sliderMachineState.armScore);
                        finger1.setPosition(sliderMachineState.stabFinger1Tight);
                        finger2.setPosition(sliderMachineState.stabFinger2Tight);
                        wrist.setPosition(sliderMachineState.wristScore);

                    })

                    .waitSeconds(.1)


                    //fast to board
                    .splineTo(new Vector2d(48.6, 33), Math.toRadians(0))

                    .lineToSplineHeading(new Pose2d(52, 33, Math.toRadians(-2)),
                            SampleMecanumDrive.getVelocityConstraint(15, DriveConstants.MAX_ANG_VEL,
                                    DriveConstants.TRACK_WIDTH), SampleMecanumDrive.getAccelerationConstraint((DriveConstants.MAX_ACCEL)))//to board
//slow board aproch

                    .waitSeconds(.8)

                    .addTemporalMarker(() -> {
                        finger1.setPosition(sliderMachineState.Finger1Loose);
                        finger2.setPosition(sliderMachineState.Finger2Loose);
                    })


                    .waitSeconds(.5)
                    .setReversed(true)



                    .lineToSplineHeading(new Pose2d(45, 33.5, Math.toRadians(0)))
                    .waitSeconds(.5)
                    .splineTo(new Vector2d(45, 60), Math.toRadians(90),
                            SampleMecanumDrive.getVelocityConstraint(25, DriveConstants.MAX_ANG_VEL,
                                    DriveConstants.TRACK_WIDTH), SampleMecanumDrive.getAccelerationConstraint((DriveConstants.MAX_ACCEL)))

                    .addTemporalMarker(() -> {


                        moveByEncoder.powerSlider(slides, sliderMachineState.THREATENINGpos);
                        arm1.setPosition(sliderMachineState.armThreaten);
                        arm2.setPosition(sliderMachineState.armThreaten);

                        wrist.setPosition(sliderMachineState.wristThreaten);

                    })
                    .strafeLeft(9)

                    .waitSeconds(2)


                    .build();

            drive.followTrajectorySequence(trajectory1);


        } else if (zone == 3) {
            telemetry.addLine("running zone 3 auto!");
            telemetry.update();




            TrajectorySequence trajectory1 = drive.trajectorySequenceBuilder(startPose)

                    .addTemporalMarker(() -> {
                        flicker.setPosition(.5);
                        finger1.setPosition(sliderMachineState.stabFinger1Tight);
                        finger2.setPosition(sliderMachineState.stabFinger2Tight);
                    })


                    .lineToSplineHeading(new Pose2d(-39, 54.4, Math.toRadians(-90)))


                    .lineToSplineHeading(new Pose2d(-38.67, 32.4, Math.toRadians(180)))

                    //place pixel
                    .addTemporalMarker(() -> {flicker.setPosition(.8);} )

                    .waitSeconds(.1)


                    .lineToSplineHeading(new Pose2d(-35.78, 32.4, Math.toRadians(180)))//escape pixel

                    .lineToSplineHeading(new Pose2d(-37, 41.8, Math.toRadians(180)))

                    .lineToSplineHeading(new Pose2d(-38, 41.8, Math.toRadians(-90)))//turn

                    .lineToSplineHeading(new Pose2d(-37.5, 58.11, Math.toRadians(0)))

                    //out of truss
                    .lineToSplineHeading(new Pose2d(17.4, 58.1, Math.toRadians(0)))

                    .waitSeconds(.1)
                    .addTemporalMarker(() -> {


                        moveByEncoder.powerSlider(slides, sliderMachineState.LOWpos + 200);
                        arm1.setPosition(sliderMachineState.armScore);
                        arm2.setPosition(sliderMachineState.armScore);
                        finger1.setPosition(sliderMachineState.stabFinger1Tight);
                        finger2.setPosition(sliderMachineState.stabFinger2Tight);
                        wrist.setPosition(sliderMachineState.wristScore);

                    })

                    //fast to board
                    .splineTo(new Vector2d(45.5, 27), Math.toRadians(0))

                    .waitSeconds(1)
                    //slow to board
                    .lineToSplineHeading(new Pose2d(53.5, 27, Math.toRadians(0)),
                            SampleMecanumDrive.getVelocityConstraint(15, DriveConstants.MAX_ANG_VEL,
                                    DriveConstants.TRACK_WIDTH), SampleMecanumDrive.getAccelerationConstraint((DriveConstants.MAX_ACCEL)))

                    .waitSeconds(.3)


                    .addTemporalMarker(() -> {
                        finger1.setPosition(sliderMachineState.Finger1Loose);
                        finger2.setPosition(sliderMachineState.Finger2Loose);
                    })


                    .waitSeconds(.5)
                    .setReversed(true)



                    .lineToSplineHeading(new Pose2d(45, 27, Math.toRadians(0)))
                    .splineTo(new Vector2d(45, 55), Math.toRadians(90),
                            SampleMecanumDrive.getVelocityConstraint(15, DriveConstants.MAX_ANG_VEL,
                                    DriveConstants.TRACK_WIDTH), SampleMecanumDrive.getAccelerationConstraint((DriveConstants.MAX_ACCEL)))
                    .waitSeconds(.1)
                    .lineToSplineHeading(new Pose2d(45.1, 60, Math.toRadians(-90)))


                    .addTemporalMarker(() -> {


                        moveByEncoder.powerSlider(slides, sliderMachineState.THREATENINGpos);
                        arm1.setPosition(sliderMachineState.armThreaten);
                        arm2.setPosition(sliderMachineState.armThreaten);

                        wrist.setPosition(sliderMachineState.wristThreaten);

                    })
                    .strafeLeft(9)

                    .waitSeconds(2)



                    .build();


            drive.followTrajectorySequence(trajectory1);


        } else {
            telemetry.addLine("running zone 1 auto!");
            telemetry.update();




            TrajectorySequence trajectory1 = drive.trajectorySequenceBuilder(startPose)



                    .addTemporalMarker(() -> {
                        flicker.setPosition(.5);
                        finger1.setPosition(sliderMachineState.stabFinger1Tight);
                        finger2.setPosition(sliderMachineState.stabFinger2Tight);
                    })

                    .lineToSplineHeading(new Pose2d(-44.1, 53.85, Math.toRadians(-90)))
                    .lineToSplineHeading(new Pose2d(-30.36, 34.45, Math.toRadians(0)))


                    .waitSeconds(.1)
                    .addTemporalMarker(() -> {flicker.setPosition(.8);} )

                    .waitSeconds(.1)


                    .lineToSplineHeading(new Pose2d(-45, 34.45, Math.toRadians(0)))//to avoid the pole

                    //w

                    .lineToSplineHeading(new Pose2d(-41.9, 58.65, Math.toRadians(0)))

                    .lineToSplineHeading(new Pose2d(14.7, 58.05, Math.toRadians(0)))


                    .addTemporalMarker(() -> {


                        moveByEncoder.powerSlider(slides, sliderMachineState.LOWpos + 200);
                        arm1.setPosition(sliderMachineState.armScore);
                        arm2.setPosition(sliderMachineState.armScore);
                        finger1.setPosition(sliderMachineState.stabFinger1Tight);
                        finger2.setPosition(sliderMachineState.stabFinger2Tight);
                        wrist.setPosition(sliderMachineState.wristScore);

                    })
                    .waitSeconds(.5)

                    .splineTo(new Vector2d(45, 39.2), Math.toRadians(0))
                    .splineTo(new Vector2d(51.5, 39.1), Math.toRadians(-5),
                            SampleMecanumDrive.getVelocityConstraint(15, DriveConstants.MAX_ANG_VEL,
                                    DriveConstants.TRACK_WIDTH), SampleMecanumDrive.getAccelerationConstraint((DriveConstants.MAX_ACCEL)))//to board
//slow board aproch

                    .waitSeconds(.5)

                    .addTemporalMarker(() -> {
                        finger1.setPosition(sliderMachineState.Finger1Loose);
                        finger2.setPosition(sliderMachineState.Finger2Loose);
                    })

                    .waitSeconds(.5)

                    .lineToSplineHeading(new Pose2d(45, 39.1, Math.toRadians(0)))
                    /*.splineTo(new Vector2d(45, 59), Math.toRadians(90),
                            SampleMecanumDrive.getVelocityConstraint(15, DriveConstants.MAX_ANG_VEL,
                                    DriveConstants.TRACK_WIDTH), SampleMecanumDrive.getAccelerationConstraint((DriveConstants.MAX_ACCEL)))
*/
                    .setReversed(true)
                    .splineTo(new Vector2d(45, 59), Math.toRadians(90),
                            SampleMecanumDrive.getVelocityConstraint(15, DriveConstants.MAX_ANG_VEL,
                                    DriveConstants.TRACK_WIDTH), SampleMecanumDrive.getAccelerationConstraint((DriveConstants.MAX_ACCEL)))



                    .waitSeconds(.1)

                    .addTemporalMarker(() -> {


                        moveByEncoder.powerSlider(slides, sliderMachineState.THREATENINGpos);
                        arm1.setPosition(sliderMachineState.armThreaten);
                        arm2.setPosition(sliderMachineState.armThreaten);

                        wrist.setPosition(sliderMachineState.wristThreaten);

                    })
                    .strafeLeft(9)

                    .waitSeconds(2)


                    .build();


            drive.followTrajectorySequence(trajectory1);
        }



        /*while(!isStopRequested()){
            PoseStorage.currentPose = drive.getPoseEstimate();
        }*/


    }


}