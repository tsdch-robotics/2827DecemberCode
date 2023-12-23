package org.firstinspires.ftc.teamcode.drive.Autonomous.My4Auto;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
//sufffdisusdouifhsdofuihs
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
public class RedRightSide extends LinearOpMode {
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

    private static double ID1spikeMarkX = 28;
    private static double ID1spikeMarkY = 6.5;

    private static double ID1spikeMarkDegrees = 5;

    private static double ID2spikeMarkX = 29;
    private static double ID2spikeMarkY = -4;

    private static double ID2spikeMarkDegrees = 0;

    private static double ID3spikeMarkX = 29;
    private static double ID3spikeMarkY = -12;

    private static double ID3spikeMarkDegrees = -5;

    private static double sensitivityLevel = .015;//lower for higher sensitivity, raise for less sensitity


    private static double ID4X = 32;
    private static double ID5X = 25;
    private static double ID6X = 16;

    private double spikeMarkX;
    private double spikeMarkY;
    private double spikeMarkDegrees;

    private static double inchesToBoard = -40;

    private double zoneSpecificX;










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

        Pose2d startPose = new Pose2d(0, 0, Math.toRadians(0));
        drive.setPoseEstimate(startPose);



        //Pose2d afterSensing = new Pose2d(0, -10, Math.toRadians(0));



        drive.setPoseEstimate(startPose);



        WebcamName webcamName = hardwareMap.get(WebcamName.class, "webcam1");
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

// calculate average color values
        double averageLeft = totalLeftAvg / frameCount;
        double averageRight = totalRightAvg / frameCount;

        telemetry.addLine("done computing");
        telemetry.addData("left", averageLeft); // fix the display hereesdf
        telemetry.addData("right", averageRight);
        telemetry.update();
   //     sleep(5000);
//end of sensing stuff

        if (averageRight < averageLeft && (((averageLeft+averageRight)/2)*sensitivityLevel) <= (Math.abs(averageRight - averageLeft))) {
            //zone = 2;
            telemetry.addLine("Middle");
            telemetry.update();

            zoneSpecificX = ID5X;
            spikeMarkX = ID2spikeMarkX;
            spikeMarkY = ID2spikeMarkY;
            spikeMarkDegrees = ID2spikeMarkDegrees;

        } else if (averageRight > averageLeft&& (((averageLeft+averageRight)/2)*sensitivityLevel) <= (Math.abs(averageRight - averageLeft))) {

            //zone = 3;
            telemetry.addLine("Right");
            telemetry.update();
            zoneSpecificX = ID6X;
            spikeMarkX = ID3spikeMarkX;
            spikeMarkY = ID3spikeMarkY;
            spikeMarkDegrees = ID3spikeMarkDegrees;

        } else {
            //zone = 1;
            telemetry.addLine("Left");
            telemetry.update();

            zoneSpecificX = ID4X;
            spikeMarkX = ID1spikeMarkX;
            spikeMarkY = ID1spikeMarkY;
            spikeMarkDegrees = ID1spikeMarkDegrees;

        }




        //beginning of trajectories
        Trajectory traj1 = drive.trajectoryBuilder(startPose)
                .splineTo(new Vector2d(spikeMarkX, spikeMarkY), Math.toRadians(spikeMarkDegrees))
                .build();
        Trajectory traj2 = drive.trajectoryBuilder(traj1.end())
                .lineToLinearHeading(new Pose2d(24, 0, Math.toRadians(spikeMarkDegrees)))
                .build();
        Trajectory traj3 = drive.trajectoryBuilder(traj2.end())
                .lineToLinearHeading(new Pose2d(zoneSpecificX, inchesToBoard, Math.toRadians(-90)))
                .build();
       /* Trajectory traj3 = drive.trajectoryBuilder(traj2.end())
                .lineToLinearHeading(new Pose2d(zoneSpecific, inchesToBoard, Math.toRadians(-90)))//-80
                // .splineTo(new Vector2d(0, 0), Math.toRadians(-90))
                .build();
*/

        drive.followTrajectory(traj1);
        sleep(500);
        flicker.setPosition(0);
        sleep(1000);
        drive.followTrajectory(traj2);
        drive.followTrajectory(traj3);
     //   drive.followTrajectory(traj4);
        sleep(4000);
























    }



    class ExamplePipeline extends OpenCvPipeline {

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
            Rect leftRect = new Rect(200, 100, 400, 500);
            Rect rightRect = new Rect(800, 100, 400, 500);//midile is 640

            input.copyTo(outPut);
            Imgproc.rectangle(outPut, leftRect, rectColor, 20);
            Imgproc.rectangle(outPut, rightRect, rectColor, 20);

            leftCrop = YCbCr.submat(leftRect);
            rightCrop = YCbCr.submat(rightRect);

            Core.extractChannel(leftCrop, leftCrop, 1);
            Core.extractChannel(rightCrop, rightCrop, 1);

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
