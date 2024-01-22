package org.firstinspires.ftc.teamcode.drive.Autonomous.My4Auto.RedAutos;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
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
@Disabled
@Autonomous(group = "drive")
public class RedRightBroke extends LinearOpMode {

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

        Mat input = new Mat();
        Mat output = new Mat();

        Scalar redLower = new Scalar(0, 50, 100);
        Scalar redUpper = new Scalar(10, 255, 255);
        Scalar yellowLower = new Scalar(20, 50, 50);
        Scalar yellowUpper = new Scalar(40, 255, 255);
        //Scalar blueLower = new Scalar(90, 50, 50);
        //Scalar blueUpper = new Scalar(130, 255, 255);

        private Rect rect1 = new Rect(50, 150, 300, 150);
        private Rect rect2 = new Rect(440, 120, 190, 180);

        private int redPixels1;
        private int redPixels2;

        public int getBluePixels1() {
            return redPixels1;
        }

        public int getBluePixels2() {
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
            input.copyTo(this.input);

            this.input.copyTo(this.output);

            Mat roi1 = this.input.submat(rect1);
            Mat roi2 = this.input.submat(rect2);

            redPixels1 = countBluePixels(roi1);
            redPixels2 = countBluePixels(roi2);

            telemetry.addData("ProcessFrame Called", true);
            telemetry.addData("Blue Pixels in Rectangle 1", redPixels1);
            telemetry.addData("Blue Pixels in Rectangle 2", redPixels2);
            telemetry.addData("Rectangle1 Area", rect1.area());
            telemetry.addData("Rectangle2 Area", rect2.area());

            Imgproc.rectangle(this.output, rect1, new Scalar(255, 0, 0), 2);
            Imgproc.rectangle(this.output, rect2, new Scalar(255, 0, 0), 2);

            return this.output;
        }


        private int countBluePixels(Mat image) {
            Imgproc.cvtColor(image, image, Imgproc.COLOR_RGB2HSV);

            Mat blueMask = new Mat();
            Core.inRange(image, redLower, redUpper, blueMask);

            return Core.countNonZero(blueMask);
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
        telemetry.update();


        //executeSlides.magicalMacro(slides, arm1, arm2, wrist, finger1, finger2, sliderMachineState.slidePosition.STAB, slidesTime, true);


        sleep(2000);
        waitForStart();
        telemetry.addLine("started");
        telemetry.update();

        for (int i = 0; i < 25; i++) {
            telemetry.addLine("Measuring Camera stream");
            telemetry.addData("totalLeft", totalLeft);
            telemetry.update();

            // Process frames and accumulate color values
            totalLeft += colorAnalysisPipeline.getBluePixels1();
            totalRight += colorAnalysisPipeline.getBluePixels2();

            frameCount = frameCount + 1;
            sleep(10);
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
            telemetry.addLine("running zone 5 auto!");
            telemetry.update();


            TrajectorySequence trajectory1 = drive.trajectorySequenceBuilder(startPose)


                    .addTemporalMarker(() -> {
                        finger1.setPosition(sliderMachineState.stabFinger1Tight);
                        finger2.setPosition(sliderMachineState.stabFinger2Tight);
                    })



                    .splineTo(new Vector2d(13, -30), Math.toRadians(90))

                    .waitSeconds(1)
                    //place purple
                    .waitSeconds(1)


                    .lineToSplineHeading(new Pose2d(13, -46, Math.toRadians(0)))

                    .waitSeconds(.1)
                    //raise lift

                    .splineTo(new Vector2d(51, -39), Math.toRadians(0))//slow to board



                    .splineTo(new Vector2d(51.21, -35.71), Math.toRadians(0),
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



                    // .splineTo(new Vector2d(50, 40), Math.toRadians(0))

                    .splineTo(new Vector2d(45, -59), Math.toRadians(-90))
                    .waitSeconds(1)


                    .build();

            drive.followTrajectorySequence(trajectory1);
        } else if (zone == 3) {
            telemetry.addLine("running zone 6 auto!");
            telemetry.update();


            TrajectorySequence trajectory1 = drive.trajectorySequenceBuilder(startPose)

                    .addTemporalMarker(() -> {
                        finger1.setPosition(sliderMachineState.stabFinger1Tight);
                        finger2.setPosition(sliderMachineState.stabFinger2Tight);
                    })





                    .splineTo(new Vector2d(29, -30), Math.toRadians(180))

                    .waitSeconds(1)
                    //place purple
                    .waitSeconds(1)


                    .lineToSplineHeading(new Pose2d(36, -31, Math.toRadians(180)))


                    .lineToSplineHeading(new Pose2d(37, -31, Math.toRadians(0)))

                    //raise lift

                    .addTemporalMarker(() -> {


                        moveByEncoder.powerSlider(slides, sliderMachineState.LOWpos);
                        arm1.setPosition(sliderMachineState.armScore);
                        arm2.setPosition(sliderMachineState.armScore);
                        finger1.setPosition(sliderMachineState.stabFinger1Tight);
                        finger2.setPosition(sliderMachineState.stabFinger2Tight);
                        wrist.setPosition(sliderMachineState.wristScore);

                    })


                    //.lineToSplineHeading(new Pose2d(51, -39, Math.toRadians(0)))//slow to board
                    .splineTo(new Vector2d(50.6, -42), Math.toRadians(0),
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



                    // .splineTo(new Vector2d(50, 40), Math.toRadians(0))

                    .splineTo(new Vector2d(45, -59), Math.toRadians(-90))
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

                    .splineTo(new Vector2d(7, -33), Math.toRadians(180))

                    .waitSeconds(1)
                    //place purple
                    .waitSeconds(1)


                    .lineToSplineHeading(new Pose2d(17, -33, Math.toRadians(180)))


                    .lineToSplineHeading(new Pose2d(37, -31, Math.toRadians(180)))

                    //raise lift

                    .addTemporalMarker(() -> {


                        moveByEncoder.powerSlider(slides, sliderMachineState.LOWpos);
                        arm1.setPosition(sliderMachineState.armScore);
                        arm2.setPosition(sliderMachineState.armScore);
                        finger1.setPosition(sliderMachineState.stabFinger1Tight);
                        finger2.setPosition(sliderMachineState.stabFinger2Tight);
                        wrist.setPosition(sliderMachineState.wristScore);

                    })

                    .lineToSplineHeading(new Pose2d(37.1, -31.1, Math.toRadians(0)))//turn toward sboard

                    .splineTo(new Vector2d(52, -30.2), Math.toRadians(0),
                            SampleMecanumDrive.getVelocityConstraint(10, DriveConstants.MAX_ANG_VEL,
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


                    // .splineTo(new Vector2d(50, 40), Math.toRadians(0))

                    .splineTo(new Vector2d(45, -59), Math.toRadians(-90),
                            SampleMecanumDrive.getVelocityConstraint(15, DriveConstants.MAX_ANG_VEL,
                                    DriveConstants.TRACK_WIDTH), SampleMecanumDrive.getAccelerationConstraint((DriveConstants.MAX_ACCEL)))
                    .addTemporalMarker(() -> {
                        moveByEncoder.powerSlider(slides, sliderMachineState.THREATENINGpos);
                        arm1.setPosition(sliderMachineState.armThreaten);
                        arm2.setPosition(sliderMachineState.armThreaten);
                        finger1.setPosition(sliderMachineState.Finger1Loose);
                        finger2.setPosition(sliderMachineState.Finger2Loose);
                        wrist.setPosition(sliderMachineState.wristThreaten);

                    })
                    .waitSeconds(2)

                    .build();


            drive.followTrajectorySequence(trajectory1);
        }
    }


}
