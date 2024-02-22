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
public class BlueLeftPullStack extends LinearOpMode {

    private Servo intakeLeft;
    private Servo intakeRight;
    private DcMotor intake;

    private BNO055IMU imu;


    public ElapsedTime slidesTime = new ElapsedTime();
    sliderMachineState executeSlides = new sliderMachineState();

    OpenCvWebcam webcam1 = null;
    ElapsedTime elapsedTime = new ElapsedTime(); // Add ElapsedTime to track time

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
        Pose2d startPose = new Pose2d(10, 60, Math.toRadians(-90));//should be 9
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



/*
        BNO055IMU.Parameters imuParams = new BNO055IMU.Parameters();
        imuParams.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        imuParams.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        //imuParams.LOGO_FACING_DIR, USB_FACING_DIR
        imuParams.calibrationDataFile = "BNO055IMUCalibration.json"; // Set this to your calibration file
        imuParams.loggingEnabled = true;
        imuParams.loggingTag = "IMU";
        imuParams.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();
        imu = hardwareMap.get(BNO055IMU.class, "imu");


        imu.initialize(imuParams);
*/


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


            TrajectorySequence trajectory1 = drive.trajectorySequenceBuilder(startPose)

                    .addTemporalMarker(() -> {
                        flicker.setPosition(.5);
                        finger1.setPosition(sliderMachineState.stabFinger1Tight);
                        finger2.setPosition(sliderMachineState.stabFinger2Tight);
                    })

                    .addTemporalMarker(.9,() -> {

                        moveByEncoder.powerSlider(slides, sliderMachineState.LOWpos);
                        arm1.setPosition(sliderMachineState.armScore);
                        arm2.setPosition(sliderMachineState.armScore);
                        finger1.setPosition(sliderMachineState.stabFinger1Tight);
                        finger2.setPosition(sliderMachineState.stabFinger2Tight);
                        wrist.setPosition(.2/*sliderMachineState.wristScore*/);
                    })

                    .lineToSplineHeading(new Pose2d(11.5, 59, Math.toRadians(-90)))
                    .waitSeconds(.01)//
                    .splineTo(new Vector2d(45, 34), Math.toRadians(0))
                    .waitSeconds(.01)//
                    .lineToSplineHeading(new Pose2d(52, 34, Math.toRadians(0)),
                            SampleMecanumDrive.getVelocityConstraint(15, DriveConstants.MAX_ANG_VEL,
                                    DriveConstants.TRACK_WIDTH), SampleMecanumDrive.getAccelerationConstraint((DriveConstants.MAX_ACCEL)))//to board


                    .waitSeconds(.5)

                    .addTemporalMarker(() -> {
                        finger1.setPosition(sliderMachineState.Finger1Loose);
                        finger2.setPosition(sliderMachineState.Finger2Loose);
                        //release pixel
                    })

                    .waitSeconds(.5)

//slow board aproch

                    .setReversed(true)
                    .splineTo(new Vector2d(25, 26), Math.toRadians(180),
                            SampleMecanumDrive.getVelocityConstraint(50, DriveConstants.MAX_ANG_VEL,
                                    DriveConstants.TRACK_WIDTH), SampleMecanumDrive.getAccelerationConstraint((DriveConstants.MAX_ACCEL)))


                    .addTemporalMarker(3,() -> {
                        intakeLeft.setPosition(0);
                        intakeRight.setPosition(0);

                    })

                    .addTemporalMarker(() -> {
                        intake.setPower(-.4);//svore purple
                    })



                    //push pixel a lil farther
                    .lineToSplineHeading(new Pose2d(20, 26, Math.toRadians(0)),
                            SampleMecanumDrive.getVelocityConstraint(30, DriveConstants.MAX_ANG_VEL,
                                    DriveConstants.TRACK_WIDTH), SampleMecanumDrive.getAccelerationConstraint((DriveConstants.MAX_ACCEL)))


                    .waitSeconds(.01)//

                    //retract lift
                    .addTemporalMarker(() -> {
                        moveByEncoder.powerSlider(slides, 0);
                        arm1.setPosition(sliderMachineState.armThreaten);
                        arm2.setPosition(sliderMachineState.armThreaten);
                        finger1.setPosition(sliderMachineState.Finger1Loose);
                        finger2.setPosition(sliderMachineState.Finger2Loose);
                        wrist.setPosition(sliderMachineState.wristThreaten);

                    })

                    .addTemporalMarker(7,() -> {
//turn intake off after purple
                        intake.setPower(0);
                    })

                    .setReversed(false)
                    .lineToSplineHeading(new Pose2d(37, 26, Math.toRadians(0)))


                    //line up with aisle
                    .setReversed(true)
                    .splineTo(new Vector2d(30, 13), Math.toRadians(180),
                            SampleMecanumDrive.getVelocityConstraint(40, DriveConstants.MAX_ANG_VEL,
                                    DriveConstants.TRACK_WIDTH), SampleMecanumDrive.getAccelerationConstraint((DriveConstants.MAX_ACCEL)))

                    //.waitSeconds(.5)

//TRYING to Score STACK

                    .setReversed(true)
                    .splineTo(new Vector2d(20,13), Math.toRadians(180))

                    //changed the degrees
                    //.waitSeconds(.01)//


                    .lineToSplineHeading(new Pose2d(-56, 17, Math.toRadians(0)))

                    .lineToSplineHeading(new Pose2d(-62, 17, Math.toRadians(0)),
                            SampleMecanumDrive.getVelocityConstraint(10, DriveConstants.MAX_ANG_VEL,
                                    DriveConstants.TRACK_WIDTH), SampleMecanumDrive.getAccelerationConstraint((DriveConstants.MAX_ACCEL)))

                    //intake3
                    .addTemporalMarker(() -> {

                        intakeLeft.setPosition(.1);
                        intakeRight.setPosition(.1);

//turn power on for intake
                    })


                    .waitSeconds(.01)//
                    .lineToSplineHeading(new Pose2d(-55, 14, Math.toRadians(0)))

//NOW, we can turn on the intake!

                    .addTemporalMarker(() -> {

                        //create height for pixel 4
                        intakeLeft.setPosition(.35);
                        intakeRight.setPosition(.35);//.8?????
                        intake.setPower(1);

                    })

                    .waitSeconds(1)

                    //shifting over
                    .lineToSplineHeading(new Pose2d(-57, 13, Math.toRadians(0)),
                            SampleMecanumDrive.getVelocityConstraint(10, DriveConstants.MAX_ANG_VEL,
                                    DriveConstants.TRACK_WIDTH), SampleMecanumDrive.getAccelerationConstraint((DriveConstants.MAX_ACCEL)))


                    .waitSeconds(.01)//

                    .setReversed(false)
                    .splineTo(new Vector2d(-50, 13), Math.toRadians(0))//leaving the stack
                    //line to?

                    .addTemporalMarker(() -> {
                        intakeLeft.setPosition(.35);
                        intakeRight.setPosition(.35);
                        //to pixk up pixels stuck in intake
                    })

                    .waitSeconds(.01)//
                    .lineToSplineHeading(new Pose2d(30, 13, Math.toRadians(0)))//go thru gate

                    //raise lift


                    .addTemporalMarker(17,() -> {

                        moveByEncoder.powerSlider(slides, 400);

                        wrist.setPosition(sliderMachineState.wristStab);
                        arm1.setPosition(sliderMachineState.armStab);
                        arm2.setPosition(sliderMachineState.armStab);

                    })
                    .addTemporalMarker(18,() -> {

                        moveByEncoder.powerSlider(slides, -30);
                        arm1.setPosition(sliderMachineState.armStab);
                        arm2.setPosition(sliderMachineState.armStab);

                        intake.setPower(-1);

                    })
                    .addTemporalMarker(20,() -> {

                        finger1.setPosition(sliderMachineState.stabFinger1Tight);
                        finger2.setPosition(sliderMachineState.stabFinger2Tight);

                    })

                    .setReversed(false)
                    .splineTo(new Vector2d(45,35), Math.toRadians(0))

                    .addTemporalMarker(() -> {

                        moveByEncoder.powerSlider(slides, sliderMachineState.MEDIUMpos);
                        arm1.setPosition(sliderMachineState.armScore);
                        arm2.setPosition(sliderMachineState.armScore);
                        wrist.setPosition(sliderMachineState.wristScore);


                    })

                    //go slow


                    .waitSeconds(.01)//

                    .lineToSplineHeading(new Pose2d(51, 35, Math.toRadians(0)),
                            SampleMecanumDrive.getVelocityConstraint(15, DriveConstants.MAX_ANG_VEL,
                                    DriveConstants.TRACK_WIDTH), SampleMecanumDrive.getAccelerationConstraint((DriveConstants.MAX_ACCEL)))

                    .waitSeconds(1)

                    .addTemporalMarker(() -> {

                        finger1.setPosition(sliderMachineState.Finger1Loose);
                        finger2.setPosition(sliderMachineState.Finger2Loose);

                        //release pixel
                    })

                    //end  of the extra

                    .waitSeconds(1)

                    .lineToSplineHeading(new Pose2d(45, 35, Math.toRadians(0)))//back off

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

                    .addTemporalMarker(.9,() -> {

                        moveByEncoder.powerSlider(slides, sliderMachineState.LOWpos);
                        arm1.setPosition(sliderMachineState.armScore);
                        arm2.setPosition(sliderMachineState.armScore);
                        finger1.setPosition(sliderMachineState.stabFinger1Tight);
                        finger2.setPosition(sliderMachineState.stabFinger2Tight);
                        wrist.setPosition(.2/*sliderMachineState.wristScore*/);
                    })

                    .lineToSplineHeading(new Pose2d(11.5, 59, Math.toRadians(-90)))
                    .waitSeconds(.01)//
                    .splineTo(new Vector2d(45, 28), Math.toRadians(0))
                    .waitSeconds(.01)//
                    .lineToSplineHeading(new Pose2d(52, 28, Math.toRadians(0)),
                            SampleMecanumDrive.getVelocityConstraint(15, DriveConstants.MAX_ANG_VEL,
                                    DriveConstants.TRACK_WIDTH), SampleMecanumDrive.getAccelerationConstraint((DriveConstants.MAX_ACCEL)))//to board


                    .waitSeconds(.5)

                    .addTemporalMarker(() -> {
                        finger1.setPosition(sliderMachineState.Finger1Loose);
                        finger2.setPosition(sliderMachineState.Finger2Loose);
                        //release pixel
                    })

                    .waitSeconds(.5)

//slow board aproch

                    .setReversed(true)
                    .splineTo(new Vector2d(14, 35), Math.toRadians(180),
            SampleMecanumDrive.getVelocityConstraint(50, DriveConstants.MAX_ANG_VEL,
                    DriveConstants.TRACK_WIDTH), SampleMecanumDrive.getAccelerationConstraint((DriveConstants.MAX_ACCEL)))


                    .addTemporalMarker(3,() -> {
                        intakeLeft.setPosition(0);
                        intakeRight.setPosition(0);

                    })

                    .addTemporalMarker(() -> {
                        intake.setPower(-.4);//svore purple
                    })



                    //push pixel a lil farther
                    .lineToSplineHeading(new Pose2d(12, 35, Math.toRadians(0)),
            SampleMecanumDrive.getVelocityConstraint(30, DriveConstants.MAX_ANG_VEL,
                    DriveConstants.TRACK_WIDTH), SampleMecanumDrive.getAccelerationConstraint((DriveConstants.MAX_ACCEL)))


                    .waitSeconds(.01)//

                    //retract lift
                    .addTemporalMarker(() -> {
                        moveByEncoder.powerSlider(slides, 0);
                        arm1.setPosition(sliderMachineState.armThreaten);
                        arm2.setPosition(sliderMachineState.armThreaten);
                        finger1.setPosition(sliderMachineState.Finger1Loose);
                        finger2.setPosition(sliderMachineState.Finger2Loose);
                        wrist.setPosition(sliderMachineState.wristThreaten);

                    })

                    .addTemporalMarker(7,() -> {
//turn intake off after purple
                        intake.setPower(0);
                    })

                    .setReversed(false)
                    .lineToSplineHeading(new Pose2d(14, 35, Math.toRadians(0)))



            //line up with aisle
                    .splineTo(new Vector2d(30, 13), Math.toRadians(0),
                            SampleMecanumDrive.getVelocityConstraint(40, DriveConstants.MAX_ANG_VEL,
                                    DriveConstants.TRACK_WIDTH), SampleMecanumDrive.getAccelerationConstraint((DriveConstants.MAX_ACCEL)))

                    //.waitSeconds(.5)

//TRYING to Score STACK

                    .setReversed(true)
                    .splineTo(new Vector2d(20,13), Math.toRadians(180))

                    //changed the degrees
                    //.waitSeconds(.01)//


                    .lineToSplineHeading(new Pose2d(-56, 17, Math.toRadians(0)))

                    .lineToSplineHeading(new Pose2d(-62, 17, Math.toRadians(0)),
                            SampleMecanumDrive.getVelocityConstraint(10, DriveConstants.MAX_ANG_VEL,
                                    DriveConstants.TRACK_WIDTH), SampleMecanumDrive.getAccelerationConstraint((DriveConstants.MAX_ACCEL)))

                    //intake3
                    .addTemporalMarker(() -> {

                        intakeLeft.setPosition(.1);
                        intakeRight.setPosition(.1);

//turn power on for intake
                    })


                    .waitSeconds(.01)//
                    .lineToSplineHeading(new Pose2d(-55, 14, Math.toRadians(0)))

//NOW, we can turn on the intake!

                    .addTemporalMarker(() -> {

                        //create height for pixel 4
                        intakeLeft.setPosition(.35);
                        intakeRight.setPosition(.35);//.8?????
                        intake.setPower(1);

                    })

                    .waitSeconds(1)

                    //shifting over
                    .lineToSplineHeading(new Pose2d(-57, 13, Math.toRadians(0)),
                            SampleMecanumDrive.getVelocityConstraint(10, DriveConstants.MAX_ANG_VEL,
                                    DriveConstants.TRACK_WIDTH), SampleMecanumDrive.getAccelerationConstraint((DriveConstants.MAX_ACCEL)))

                    .waitSeconds(.01)//

                    .setReversed(false)
                    .splineTo(new Vector2d(-50, 13), Math.toRadians(0))//leaving the stack
                    //line to?

                    .addTemporalMarker(() -> {
                        intakeLeft.setPosition(.35);
                        intakeRight.setPosition(.35);
                        //to pixk up pixels stuck in intake
                    })

                    .waitSeconds(.01)//
                    .lineToSplineHeading(new Pose2d(30, 13, Math.toRadians(0)))//go thru gate

                    //raise lift


                    .addTemporalMarker(17,() -> {

                        moveByEncoder.powerSlider(slides, 400);

                        wrist.setPosition(sliderMachineState.wristStab);
                        arm1.setPosition(sliderMachineState.armStab);
                        arm2.setPosition(sliderMachineState.armStab);

                    })
                    .addTemporalMarker(18,() -> {

                        moveByEncoder.powerSlider(slides, -30);
                        arm1.setPosition(sliderMachineState.armStab);
                        arm2.setPosition(sliderMachineState.armStab);

                        intake.setPower(-1);

                    })
                    .addTemporalMarker(20,() -> {

                        finger1.setPosition(sliderMachineState.stabFinger1Tight);
                        finger2.setPosition(sliderMachineState.stabFinger2Tight);

                    })

                    .setReversed(false)
                    .splineTo(new Vector2d(45,35), Math.toRadians(0))

                    .addTemporalMarker(() -> {

                        moveByEncoder.powerSlider(slides, sliderMachineState.MEDIUMpos);
                        arm1.setPosition(sliderMachineState.armScore);
                        arm2.setPosition(sliderMachineState.armScore);
                        wrist.setPosition(sliderMachineState.wristScore);


                    })

                    //go slow


                    .waitSeconds(.01)//

                    .lineToSplineHeading(new Pose2d(51, 35, Math.toRadians(0)),
                            SampleMecanumDrive.getVelocityConstraint(15, DriveConstants.MAX_ANG_VEL,
                                    DriveConstants.TRACK_WIDTH), SampleMecanumDrive.getAccelerationConstraint((DriveConstants.MAX_ACCEL)))

                    .waitSeconds(1)

                    .addTemporalMarker(() -> {

                        finger1.setPosition(sliderMachineState.Finger1Loose);
                        finger2.setPosition(sliderMachineState.Finger2Loose);

                        //release pixel
                    })

                    //end  of the extra

                    .waitSeconds(1)

                    .lineToSplineHeading(new Pose2d(45, 35, Math.toRadians(0)))//back off

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

                    .addTemporalMarker(.9,() -> {

                        moveByEncoder.powerSlider(slides, sliderMachineState.LOWpos);
                        arm1.setPosition(sliderMachineState.armScore);
                        arm2.setPosition(sliderMachineState.armScore);
                        finger1.setPosition(sliderMachineState.stabFinger1Tight);
                        finger2.setPosition(sliderMachineState.stabFinger2Tight);
                        wrist.setPosition(.2/*sliderMachineState.wristScore*/);
                    })

                    .lineToSplineHeading(new Pose2d(11.5, 59, Math.toRadians(-90)))
                    .waitSeconds(.01)//

                    .splineTo(new Vector2d(45, 38.8), Math.toRadians(0))


                    .waitSeconds(.01)//
                    .lineToSplineHeading(new Pose2d(52, 38.8, Math.toRadians(0)),
                            SampleMecanumDrive.getVelocityConstraint(15, DriveConstants.MAX_ANG_VEL,
                                    DriveConstants.TRACK_WIDTH), SampleMecanumDrive.getAccelerationConstraint((DriveConstants.MAX_ACCEL)))//to board


                    .waitSeconds(.5)

                    .addTemporalMarker(() -> {
                        finger1.setPosition(sliderMachineState.Finger1Loose);
                        finger2.setPosition(sliderMachineState.Finger2Loose);
                        //release pixel
                    })

                    .waitSeconds(.5)

//slow board aproch

                    .setReversed(true)
                    .splineTo(new Vector2d(40, 35), Math.toRadians(180),
                            SampleMecanumDrive.getVelocityConstraint(50, DriveConstants.MAX_ANG_VEL,
                                    DriveConstants.TRACK_WIDTH), SampleMecanumDrive.getAccelerationConstraint((DriveConstants.MAX_ACCEL)))


                    .addTemporalMarker(3,() -> {
                        intakeLeft.setPosition(0);
                        intakeRight.setPosition(0);

                    })

                    .addTemporalMarker(() -> {
                        intake.setPower(-.4);//svore purple
                    })



                    //push pixel a lil farther
                    .lineToSplineHeading(new Pose2d(37, 35, Math.toRadians(0)),
                            SampleMecanumDrive.getVelocityConstraint(30, DriveConstants.MAX_ANG_VEL,
                                    DriveConstants.TRACK_WIDTH), SampleMecanumDrive.getAccelerationConstraint((DriveConstants.MAX_ACCEL)))


                    .waitSeconds(.01)//

                    //retract lift
                    .addTemporalMarker(() -> {
                        moveByEncoder.powerSlider(slides, 0);
                        arm1.setPosition(sliderMachineState.armThreaten);
                        arm2.setPosition(sliderMachineState.armThreaten);
                        finger1.setPosition(sliderMachineState.Finger1Loose);
                        finger2.setPosition(sliderMachineState.Finger2Loose);
                        wrist.setPosition(sliderMachineState.wristThreaten);

                    })

                    .addTemporalMarker(7,() -> {
//turn intake off after purple
                        intake.setPower(0);
                    })

                    .setReversed(false)
                    .lineToSplineHeading(new Pose2d(45, 35, Math.toRadians(0)))



                    //line up with aisle
                    .splineTo(new Vector2d(30, 13), Math.toRadians(0),
                            SampleMecanumDrive.getVelocityConstraint(40, DriveConstants.MAX_ANG_VEL,
                                    DriveConstants.TRACK_WIDTH), SampleMecanumDrive.getAccelerationConstraint((DriveConstants.MAX_ACCEL)))

                    //.waitSeconds(.5)

//TRYING to Score STACK

                    .setReversed(true)
                    .splineTo(new Vector2d(20,13), Math.toRadians(180))

                    //changed the degrees
                    //.waitSeconds(.01)//


                    .lineToSplineHeading(new Pose2d(-56, 17, Math.toRadians(0)))

                    .lineToSplineHeading(new Pose2d(-62, 17, Math.toRadians(0)),
                            SampleMecanumDrive.getVelocityConstraint(10, DriveConstants.MAX_ANG_VEL,
                                    DriveConstants.TRACK_WIDTH), SampleMecanumDrive.getAccelerationConstraint((DriveConstants.MAX_ACCEL)))

                    //intake3
                    .addTemporalMarker(() -> {

                        intakeLeft.setPosition(.1);
                        intakeRight.setPosition(.1);

//turn power on for intake
                    })


                    .waitSeconds(.01)//
                    .lineToSplineHeading(new Pose2d(-55, 14, Math.toRadians(0)))

//NOW, we can turn on the intake!

                    .addTemporalMarker(() -> {

                        //create height for pixel 4
                        intakeLeft.setPosition(.35);
                        intakeRight.setPosition(.35);//.8?????
                        intake.setPower(1);

                    })

                    .waitSeconds(1)

                    //shifting over
                    .lineToSplineHeading(new Pose2d(-57, 13, Math.toRadians(0)),
                            SampleMecanumDrive.getVelocityConstraint(10, DriveConstants.MAX_ANG_VEL,
                                    DriveConstants.TRACK_WIDTH), SampleMecanumDrive.getAccelerationConstraint((DriveConstants.MAX_ACCEL)))


                    .waitSeconds(.01)//

                    .setReversed(false)
                    .splineTo(new Vector2d(-50, 13), Math.toRadians(0))//leaving the stack
                    //line to?

                    .addTemporalMarker(() -> {
                        intakeLeft.setPosition(.35);
                        intakeRight.setPosition(.35);
                        //to pixk up pixels stuck in intake
                    })

                    .waitSeconds(.01)//
                    .lineToSplineHeading(new Pose2d(30, 13, Math.toRadians(0)))//go thru gate

                    //raise lift


                    .addTemporalMarker(17,() -> {

                        moveByEncoder.powerSlider(slides, 400);

                        wrist.setPosition(sliderMachineState.wristStab);
                        arm1.setPosition(sliderMachineState.armStab);
                        arm2.setPosition(sliderMachineState.armStab);

                    })
                    .addTemporalMarker(18,() -> {

                        moveByEncoder.powerSlider(slides, 30);
                        arm1.setPosition(sliderMachineState.armStab);
                        arm2.setPosition(sliderMachineState.armStab);

                        intake.setPower(-1);

                    })
                    .addTemporalMarker(20,() -> {

                        finger1.setPosition(sliderMachineState.stabFinger1Tight);
                        finger2.setPosition(sliderMachineState.stabFinger2Tight);

                    })

                    .setReversed(false)
                    .splineTo(new Vector2d(45,35), Math.toRadians(0))

                    .addTemporalMarker(() -> {

                        moveByEncoder.powerSlider(slides, sliderMachineState.MEDIUMpos);
                        arm1.setPosition(sliderMachineState.armScore);
                        arm2.setPosition(sliderMachineState.armScore);
                        wrist.setPosition(sliderMachineState.wristScore);


                    })

                    //go slow


                    .waitSeconds(.01)//

                    .lineToSplineHeading(new Pose2d(51, 35, Math.toRadians(0)),
                            SampleMecanumDrive.getVelocityConstraint(15, DriveConstants.MAX_ANG_VEL,
                                    DriveConstants.TRACK_WIDTH), SampleMecanumDrive.getAccelerationConstraint((DriveConstants.MAX_ACCEL)))

                    .waitSeconds(1)

                    .addTemporalMarker(() -> {

                        finger1.setPosition(sliderMachineState.Finger1Loose);
                        finger2.setPosition(sliderMachineState.Finger2Loose);

                        //release pixel
                    })

                    //end  of the extra

                    .waitSeconds(1)

                    .lineToSplineHeading(new Pose2d(45, 35, Math.toRadians(0)))//back off

                    .build();



            drive.followTrajectorySequence(trajectory1);




        }
    }


}