package org.firstinspires.ftc.teamcode.drive.Autonomous.My4Auto.BlueAutos;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.drive.TeleOp.DavidsFUNctions.sliderMachineState;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
//sufffdisusdouifhsdofuihs
import org.firstinspires.ftc.teamcode.drive.TeleOp.DavidsFUNctions.sliderMachineState;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;
import org.openftc.easyopencv.OpenCvWebcam;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvCameraFactory;

import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime; // Import ElapsedTime

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

@Config
@Autonomous(group = "drive")
public class BlueRightSide extends LinearOpMode {



    public ElapsedTime slidesTime = new ElapsedTime();
    sliderMachineState executeSlides = new sliderMachineState();



    OpenCvWebcam webcam1 = null;
    ElapsedTime elapsedTime = new ElapsedTime(); // Add ElapsedTime to track time
    double totalLeftAvg = 0;
    double totalRightAvg = 0;
  //  double left = 0;
 //   double right = 0;
    int frameCount = 0;
    int zone = 0;
    ExamplePipeline examplePipeline;
    private DcMotor slides;


    //constants



    public static Pose2d ID1Location = new Pose2d(28, 6.8, Math.toRadians(5));
    public static Pose2d ID2Location = new Pose2d(29, -4, Math.toRadians(0));
    public static Pose2d ID3Location = new Pose2d(29, -12, Math.toRadians(-5));

    public static Pose2d AprilTagScore1 = new Pose2d(12, 92, Math.toRadians(95));
    public static Pose2d AprilTagScore2 = new Pose2d(11, 92, Math.toRadians(95));
    public static Pose2d AprilTagScore3 = new Pose2d(17, 92, Math.toRadians(95));

    public static double sensitivityLevel = .075;//lower for higher sensitivity, raise for less sensitity

    public static Pose2d sensedSpikeMarkLocal;
    private static Pose2d AprilTagScore;

    public static Pose2d parkPos = new Pose2d(0, 87, Math.toRadians(0));;

    public static Rect leftRect = new Rect(200, 100, 400, 500);
    public static Rect rightRect = new Rect(800, 100, 400, 500);//midile is 640






    private Servo arm1;
    private Servo arm2;

    private Servo wrist;
    private Servo finger1;

    private Servo finger2;

    private Servo flicker;


    // public final int firstForward = 20;
    //public final int firstStrafe = -50;


    @Override
    public void runOpMode() throws InterruptedException {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        Pose2d startPose = new Pose2d(0, 0, Math.toRadians(0));//(39,-62) in terms of FTC coordinates
        drive.setPoseEstimate(startPose);



        //Pose2d afterSensing = new Pose2d(0, -10, Math.toRadians(0));

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

        examplePipeline = new ExamplePipeline();
        webcam1.setPipeline(examplePipeline);

        webcam1.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                webcam1.startStreaming(1280, 720, OpenCvCameraRotation.UPRIGHT);//changed to upside down
            }//TODO: adjust width and height baced on specific camera

            @Override
            public void onError(int errorCode) {

            }
        });
        telemetry.addLine("Waiting to start");
        telemetry.update();
        //slidesTime.reset();


        executeSlides.magicalMacro(slides, arm1, arm2, wrist, finger1, finger2, sliderMachineState.slidePosition.THREATEN, slidesTime, true);

        waitForStart();
        // Run for 10 seconds
        telemetry.addLine("started");
        telemetry.update();

//necessary to work
        for (int i = 0; i < 50; i++) { // 200 iterations * 50 milliseconds = 10 seconds, 50 * 50 = 2.5 seconds
            telemetry.addLine("Measuring Camera stream");
            telemetry.update();

            // Process frames and accumulate color values
            totalLeftAvg += examplePipeline.leftavgfin;
            totalRightAvg += examplePipeline.rightavgfin;
            frameCount++;

            // Sleep for 50 milliseconds
            sleep(50);
        }

// Calculate average color values over ten seconds
        double averageLeft = totalLeftAvg / frameCount;
        double averageRight = totalRightAvg / frameCount;

        telemetry.addLine("done computing");
        telemetry.addData("left", averageLeft); // Fix the display here
        telemetry.addData("right", averageRight);
        telemetry.update();
        sleep(500);
//end of sensing stuff

        if (averageRight > averageLeft && (((averageLeft+averageRight)/2)*sensitivityLevel) <= (Math.abs(averageRight - averageLeft))) {
            zone = 2;


            AprilTagScore = AprilTagScore2;
            sensedSpikeMarkLocal = ID2Location;


            telemetry.addData("Zone", zone);
            telemetry.addData("sensedSpikeMarkLocal", sensedSpikeMarkLocal);
            telemetry.addData("AprilTagScore", AprilTagScore);




            telemetry.update();




//middle

//adjusts negligibility by scale rather than an actual parameter
        } else if (averageRight < averageLeft&& (((averageLeft+averageRight)/2)*sensitivityLevel) <= (Math.abs(averageRight - averageLeft))) {

            zone = 3;
            telemetry.addData("Zone", zone);

//middle
            AprilTagScore = AprilTagScore3;
            sensedSpikeMarkLocal = ID3Location;

            telemetry.addData("sensedSpikeMarkLocal", sensedSpikeMarkLocal);
            telemetry.addData("AprilTagScore", AprilTagScore);

            telemetry.update();

        } else {
            zone = 1;
            telemetry.addData("Zone", zone);



            AprilTagScore = AprilTagScore1;
            sensedSpikeMarkLocal = ID1Location;

            telemetry.addData("sensedSpikeMarkLocal", sensedSpikeMarkLocal);
            telemetry.addData("AprilTagScore", AprilTagScore);

            telemetry.update();
        }

        TrajectorySequence scoreThePurple = drive.trajectorySequenceBuilder(startPose)
                .splineTo(new Vector2d(sensedSpikeMarkLocal.getX(), sensedSpikeMarkLocal.getY()), sensedSpikeMarkLocal.getHeading())
                .waitSeconds(1)
               // .dropPixel(flicker)
                .build();


        TrajectorySequence outOfTheTruss = drive.trajectorySequenceBuilder(scoreThePurple.end())

                .lineToLinearHeading(new Pose2d(24, 0, sensedSpikeMarkLocal.getHeading()))
                .waitSeconds(1)
                .lineToLinearHeading(new Pose2d(3, 0, Math.toRadians(90)))
                .lineToLinearHeading(new Pose2d(3, 50, Math.toRadians(90)))
                .build();//takes robot to the board
        TrajectorySequence toTheTag = drive.trajectorySequenceBuilder(outOfTheTruss.end())
                .lineToLinearHeading(AprilTagScore)
                .build();//takes robot to the board
        TrajectorySequence park = drive.trajectorySequenceBuilder(toTheTag.end())
                .waitSeconds(1)
                .lineToLinearHeading(parkPos)
                .build();//takes robot to the board





        executeSlides.magicalMacro(slides, arm1, arm2, wrist, finger1, finger2, sliderMachineState.slidePosition.STAB, slidesTime, true);
        sleep(2000);


        drive.followTrajectorySequence(scoreThePurple);
        flicker.setPosition(0);
        sleep(1000);

        drive.followTrajectorySequence(outOfTheTruss);
        executeSlides.magicalMacro(slides, arm1, arm2, wrist, finger1, finger2, sliderMachineState.slidePosition.LOW, slidesTime, true);
        sleep(1000);

        drive.followTrajectorySequence(toTheTag);
        finger1.setPosition(sliderMachineState.Finger1Loose);
        finger2.setPosition(sliderMachineState.Finger1Loose);
        sleep(2000);

        drive.followTrajectorySequence(park);

        executeSlides.magicalMacro(slides, arm1, arm2, wrist, finger1, finger2, sliderMachineState.slidePosition.THREATEN, slidesTime, true);


        sleep(4000);

    }


    public class ExamplePipeline extends OpenCvPipeline {

        Mat YCbCr = new Mat();
        Mat leftCrop;
        Mat rightCrop;
        double leftavgfin;
        double rightavgfin;
        Mat outPut = new Mat();

        Scalar rectColor = new Scalar(0.0, 0.0, 255.0);

        @Override
        public Mat processFrame(Mat input) {
            Imgproc.cvtColor(input, YCbCr, Imgproc.COLOR_RGB2YCrCb);
//1280,720
            //Rect leftRect = new Rect(200, 100, 400, 500);
            //Rect rightRect = new Rect(800, 100, 400, 500);//midile is 640

            input.copyTo(outPut);
            Imgproc.rectangle(outPut, leftRect, rectColor, 20);
            Imgproc.rectangle(outPut, rightRect, rectColor, 20);

            leftCrop = YCbCr.submat(leftRect);
            rightCrop = YCbCr.submat(rightRect);

            Core.extractChannel(leftCrop, leftCrop, 0);
            Core.extractChannel(rightCrop, rightCrop, 0);

            Scalar leftavg = Core.mean(leftCrop);
            Scalar rightavg = Core.mean(rightCrop);

            leftavgfin = leftavg.val[0];
            rightavgfin = rightavg.val[0];

     /*       telemetry.addLine("pipeline running");
            telemetry.addData("LeftValue", leftavgfin);
            telemetry.addData("RightValue", rightavgfin);
*/
            return outPut;
        }
    }





}
