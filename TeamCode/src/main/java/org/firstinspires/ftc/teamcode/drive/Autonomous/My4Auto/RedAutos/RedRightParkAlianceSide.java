package org.firstinspires.ftc.teamcode.drive.Autonomous.My4Auto.RedAutos;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;
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
@Autonomous(group = "drive", preselectTeleOp = "TeleOp2")
public class RedRightParkAlianceSide extends LinearOpMode {


    TouchSensor touchSensor;

    public ElapsedTime slidesTime = new ElapsedTime();
    sliderMachineState executeSlides = new sliderMachineState();


    sliderMachineState.slidePosition executePos = sliderMachineState.slidePosition.THREATEN;

    OpenCvWebcam webcam1 = null;
    ElapsedTime elapsedTime = new ElapsedTime(); // Add ElapsedTime to track time

    public ElapsedTime scoreWaitingTime = new ElapsedTime();

    moveWithBasicEncoder moveByEncoder = new moveWithBasicEncoder();

    int frameCount = 0;
    int zone = 0;
    int totalLeft;
    int totalRight;

    public boolean greaterThanTargetPercentRedPixels1 = false;
    public boolean greaterThanTargetPercentRedPixels2 = false;

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

        Mat redMask = new Mat();

        Mat inputMat = new Mat();
        Mat output = new Mat();




        Scalar redLower = new Scalar(0, 50, 50);
        Scalar redUpper = new Scalar(20, 255, 255);
        Scalar yellowLower = new Scalar(20, 50, 50);
        Scalar yellowUpper = new Scalar(40, 255, 255);
        //Scalar blueLower = new Scalar(90, 50, 50);
        //Scalar blueUpper = new Scalar(130, 255, 255);


        private Rect rect1 = new Rect(50, 100, 250, 150);
        private Rect rect2 = new Rect(440, 120, 190, 180);

        //private Rect rect1 = new Rect(50, 150, 300, 150);
        //private Rect rect2 = new Rect(440, 120, 190, 180);

        private int redPixels1;
        private int redPixels2;

        public int getRedPixels1() {
            return redPixels1;
        }

        public int getRedPixels2() {
            return redPixels2;
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

            redPixels1 = countRedPixels(roi1);
            redPixels2 = countRedPixels(roi2);

            telemetry.addData("ProcessFrame Called", true);
            telemetry.addData("Red Pixels in Rectangle 1", redPixels1);
            telemetry.addData("Red Pixels in Rectangle 2", redPixels2);
            telemetry.addData("Rectangle1 Area", rect1.area());

            telemetry.addData("Rectangle2 Area", rect2.area());

            Imgproc.rectangle(this.output, rect1, new Scalar(255, 0, 0), 2);
            Imgproc.rectangle(this.output, rect2, new Scalar(255, 0, 0), 2);


            roi1.release();
            roi2.release();

            return this.output;
        }


        private int countRedPixels(Mat image) {
            Imgproc.cvtColor(image, image, Imgproc.COLOR_RGB2HSV);

            //Mat blueMask = new Mat();
            Core.inRange(image, redLower, redUpper, redMask);

            return Core.countNonZero(redMask);
        }
    }

    @Override
    public void runOpMode() throws InterruptedException {



        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        Pose2d startPose = new Pose2d(14.5, -60, Math.toRadians(90));
        drive.setPoseEstimate(startPose);

        drive.setPoseEstimate(startPose);





        WebcamName webcamName = hardwareMap.get(WebcamName.class, "Webcam 1");
        int cameraMonitorViewId = hardwareMap.appContext.getResources()
                .getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam1 = OpenCvCameraFactory.getInstance().createWebcam(webcamName, cameraMonitorViewId);




        touchSensor = hardwareMap.get(TouchSensor.class, "touch");






        slides = hardwareMap.dcMotor.get("slides");
        slides.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        slides.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        arm1 = hardwareMap.servo.get("arm1");
        arm2 = hardwareMap.servo.get("arm2");
        wrist = hardwareMap.servo.get("wrist");
        finger1 = hardwareMap.servo.get("finger1");
        finger2 = hardwareMap.servo.get("finger2");
        flicker = hardwareMap.servo.get("flicker");

        arm1.setDirection(Servo.Direction.REVERSE);

        scoreWaitingTime.reset();
        scoreWaitingTime.startTime();

        slidesTime.reset();
        slidesTime.startTime();





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

        telemetry.addData("RedPixels1", colorAnalysisPipeline.getRedPixels1());
        telemetry.addData("RedPixels1", colorAnalysisPipeline.getRedPixels2());
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
            totalLeft += colorAnalysisPipeline.getRedPixels1();
            totalRight += colorAnalysisPipeline.getRedPixels2();

            frameCount = frameCount + 1;
            sleep(0);
        }

        webcam1.stopStreaming();//may want to remove

        double averageLeft = totalLeft / frameCount;
        double averageRight = totalRight / frameCount;

        if (averageLeft / colorAnalysisPipeline.getRect1().area() > targetPixPercent1) {
            greaterThanTargetPercentRedPixels1 = true;
        } else if (averageRight / colorAnalysisPipeline.getRect2().area() > targetPixPercent2) {
            greaterThanTargetPercentRedPixels2 = true;
        } else {
            greaterThanTargetPercentRedPixels1 = false;
            greaterThanTargetPercentRedPixels2 = false;
        }

        if (averageLeft > averageRight && greaterThanTargetPercentRedPixels1) {
            telemetry.addLine("Middle");
            zone = 2;
        } else if (averageLeft < averageRight && greaterThanTargetPercentRedPixels2) {
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
            telemetry.addLine("running zone 5 auto!");
            telemetry.update();




            TrajectorySequence trajectory1 = drive.trajectorySequenceBuilder(startPose)


                    .addTemporalMarker(() -> {
                        finger1.setPosition(sliderMachineState.stabFinger1Tight);
                        finger2.setPosition(sliderMachineState.stabFinger2Tight);
                    })


                    .splineTo(new Vector2d(13, -30), Math.toRadians(90))//13

                    .waitSeconds(1)
                    //place purple
                    .addTemporalMarker(() -> {
                        flicker.setPosition(.8);
                    })
                    .waitSeconds(1)

                    .lineToSplineHeading(new Pose2d(13, -38, Math.toRadians(90)))
                    .waitSeconds(.01)
                    .lineToSplineHeading(new Pose2d(13.1, -38, Math.toRadians(0)))
                    .waitSeconds(.1)

                    //raise lift
                    .addTemporalMarker(() -> {


                        moveByEncoder.powerSlider(slides, sliderMachineState.LOWpos);
                        arm1.setPosition(sliderMachineState.armScore);
                        arm2.setPosition(sliderMachineState.armScore);
                        finger1.setPosition(sliderMachineState.stabFinger1Tight);
                        finger2.setPosition(sliderMachineState.stabFinger2Tight);
                        wrist.setPosition(sliderMachineState.wristScore);

                    })

                    .splineTo(new Vector2d(45, -35), Math.toRadians(0))
                    .splineTo(new Vector2d(51, -35), Math.toRadians(0),
            SampleMecanumDrive.getVelocityConstraint(15, DriveConstants.MAX_ANG_VEL,
                    DriveConstants.TRACK_WIDTH), SampleMecanumDrive.getAccelerationConstraint((DriveConstants.MAX_ACCEL)))
            //slow to park//slow to board, middle


                    .waitSeconds(1)
                    //score
                    .addTemporalMarker(() -> {

                        finger1.setPosition(sliderMachineState.Finger1Loose);
                        finger2.setPosition(sliderMachineState.Finger2Loose);
                        //release pixel
                    })


                    .waitSeconds(1)

                    .lineToSplineHeading(new Pose2d(45, -34, Math.toRadians(0)))//back off


                    .setReversed(true)



                    // .splineTo(new Vector2d(50, 40), Math.toRadians(0))

                    .splineTo(new Vector2d(45, -59), Math.toRadians(-90),
            SampleMecanumDrive.getVelocityConstraint(15, DriveConstants.MAX_ANG_VEL,
                    DriveConstants.TRACK_WIDTH), SampleMecanumDrive.getAccelerationConstraint((DriveConstants.MAX_ACCEL)))
            //slow to park
                    .waitSeconds(1)


                    .build();




            drive.followTrajectorySequence(trajectory1);



//TODO: do not touich ozned 6!
        } else if (zone == 3) {
            telemetry.addLine("running zone 6 auto!");
            telemetry.update();

            TrajectorySequence trajectory1 = drive.trajectorySequenceBuilder(startPose)


                    .addTemporalMarker(() -> {
                        finger1.setPosition(sliderMachineState.stabFinger1Tight);
                        finger2.setPosition(sliderMachineState.stabFinger2Tight);
                    })



                    .splineTo(new Vector2d(30.5, -30), Math.toRadians(180))

                    .waitSeconds(1)
                    //place purple
                    .addTemporalMarker(() -> {
                        flicker.setPosition(.8);
                    })
                    .waitSeconds(1)


                    .lineToSplineHeading(new Pose2d(45, -31.1, Math.toRadians(180)))

                    .waitSeconds(.5)

                    .lineToSplineHeading(new Pose2d(45, -31, Math.toRadians(0)))

                    //raise lift

                    .addTemporalMarker(() -> {


                        moveByEncoder.powerSlider(slides, sliderMachineState.LOWpos);
                        arm1.setPosition(sliderMachineState.armScore);
                        arm2.setPosition(sliderMachineState.armScore);
                        finger1.setPosition(sliderMachineState.stabFinger1Tight);
                        finger2.setPosition(sliderMachineState.stabFinger2Tight);
                        wrist.setPosition(sliderMachineState.wristScore);

                    })


                    .lineToSplineHeading(new Pose2d(45, -40.5, Math.toRadians(0)))
                    .waitSeconds(.1)
                    .splineTo(new Vector2d(51.3, -40.5), Math.toRadians(0),
                            SampleMecanumDrive.getVelocityConstraint(15, DriveConstants.MAX_ANG_VEL,
                                    DriveConstants.TRACK_WIDTH), SampleMecanumDrive.getAccelerationConstraint((DriveConstants.MAX_ACCEL)))
//slow to board


                    .waitSeconds(1)
                    //score


                    .addTemporalMarker(() -> {

                        finger1.setPosition(sliderMachineState.Finger1Loose);
                        finger2.setPosition(sliderMachineState.Finger2Loose);
                        //release pixel
                    })


                    .waitSeconds(1)
                    .setReversed(true)


                    .lineToSplineHeading(new Pose2d(45, -42, Math.toRadians(0)))//back off


                    .splineTo(new Vector2d(45, -59), Math.toRadians(-90),
                            SampleMecanumDrive.getVelocityConstraint(15, DriveConstants.MAX_ANG_VEL,
                                    DriveConstants.TRACK_WIDTH), SampleMecanumDrive.getAccelerationConstraint((DriveConstants.MAX_ACCEL)))
                    //slow to park
                    .waitSeconds(1)



                    .build();


            drive.followTrajectorySequence(trajectory1);


        } else {
            telemetry.addLine("running zone 4 auto!");
            telemetry.update();




            TrajectorySequence trajectory1 = drive.trajectorySequenceBuilder(startPose)


                    .addTemporalMarker(() -> {
                        finger1.setPosition(sliderMachineState.stabFinger1Tight);
                        finger2.setPosition(sliderMachineState.stabFinger2Tight);
                    })

                    .splineTo(new Vector2d(15, -33), Math.toRadians(180))
                    .waitSeconds(.1)
                    .lineToSplineHeading(new Pose2d(7, -33, Math.toRadians(180)))

                    .waitSeconds(1)
                    //place purple
                    .addTemporalMarker(() -> {
                        flicker.setPosition(.8);
                    })
                    .waitSeconds(1)

                    .lineToSplineHeading(new Pose2d(18, -33.01, Math.toRadians(180)))
                    .lineToSplineHeading(new Pose2d(18, -33, Math.toRadians(0.1)))

                    .waitSeconds(.1)
                    //raise lift
                    .addTemporalMarker(() -> {


                        moveByEncoder.powerSlider(slides, sliderMachineState.LOWpos);
                        arm1.setPosition(sliderMachineState.armScore);
                        arm2.setPosition(sliderMachineState.armScore);
                        finger1.setPosition(sliderMachineState.stabFinger1Tight);
                        finger2.setPosition(sliderMachineState.stabFinger2Tight);
                        wrist.setPosition(sliderMachineState.wristScore);

                    })


                    .splineTo(new Vector2d(45, -29.1), Math.toRadians(0))
                    .waitSeconds(.01)
                    .splineTo(new Vector2d(51, -29.2), Math.toRadians(0),
            SampleMecanumDrive.getVelocityConstraint(15, DriveConstants.MAX_ANG_VEL,
                    DriveConstants.TRACK_WIDTH), SampleMecanumDrive.getAccelerationConstraint((DriveConstants.MAX_ACCEL)))
            //slow to board


                    .waitSeconds(1)
                    //score
                    .addTemporalMarker(() -> {

                        finger1.setPosition(sliderMachineState.Finger1Loose);
                        finger2.setPosition(sliderMachineState.Finger2Loose);
                        //release pixel
                    })


                    .waitSeconds(1)

                    .lineToSplineHeading(new Pose2d(45, -28, Math.toRadians(0.1)))//back off



                    .setReversed(true)



                    // .splineTo(new Vector2d(50, 40), Math.toRadians(0))

                    .splineTo(new Vector2d(45, -59), Math.toRadians(-90),
                            SampleMecanumDrive.getVelocityConstraint(15, DriveConstants.MAX_ANG_VEL,
                                    DriveConstants.TRACK_WIDTH), SampleMecanumDrive.getAccelerationConstraint((DriveConstants.MAX_ACCEL)))
                    //slow to park

                    .waitSeconds(1)


                    .build();


            drive.followTrajectorySequence(trajectory1);
        }

        /*while(!isStopRequested()){
            PoseStorage.currentPose = drive.getPoseEstimate();
        }*/


    }



}