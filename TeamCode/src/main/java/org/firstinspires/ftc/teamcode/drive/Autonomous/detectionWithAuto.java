package org.firstinspires.ftc.teamcode.drive.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
//sufffdisusdouifhsdofuihs
import org.firstinspires.ftc.teamcode.util.RedDetecter;
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

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime; // Import ElapsedTime

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvPipeline;
import org.openftc.easyopencv.OpenCvWebcam;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

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

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareDevice;

import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.util.Hardware;
import org.firstinspires.ftc.teamcode.util.RedDetecter;
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

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.openftc.easyopencv.OpenCvWebcam;


import com.acmerobotics.dashboard.config.Config;
import static org.firstinspires.ftc.teamcode.drive.DriveConstants.firstMove;
import static org.firstinspires.ftc.teamcode.drive.DriveConstants.firstStrafe;


@Autonomous(group = "drive")
public class detectionWithAuto extends LinearOpMode {
    OpenCvWebcam webcam1 = null;
    ElapsedTime elapsedTime = new ElapsedTime(); // Add ElapsedTime to track time
    double totalLeftAvg = 0;
    double totalRightAvg = 0;
    double left = 0;
    double right = 0;
    int frameCount = 0;
    int zone = 0;
    ExamplePipeline examplePipeline;
    private DcMotor slides;

    int spikeMarkX = 0;
    int spikeMarkY = 0;

    int spikeMarkDegrees = 0;

    private Servo servo1;
    private Servo servo2;
    private Servo servo3;


    // public final int firstForward = 20;
    //public final int firstStrafe = -50;


    @Override
    public void runOpMode() throws InterruptedException {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        Pose2d startPose = new Pose2d(0, 0, Math.toRadians(0));
        Pose2d afterSensing = new Pose2d(0, -10, Math.toRadians(0));



        drive.setPoseEstimate(startPose);



        WebcamName webcamName = hardwareMap.get(WebcamName.class, "webcam1");
        int cameraMonitorViewId = hardwareMap.appContext.getResources()
                .getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam1 = OpenCvCameraFactory.getInstance().createWebcam(webcamName, cameraMonitorViewId);


        slides = hardwareMap.dcMotor.get("slides");
        slides.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        slides.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        servo1 = hardwareMap.servo.get("servo1");
        servo2 = hardwareMap.servo.get("servo2");
        servo3 = hardwareMap.servo.get("servo3");

        examplePipeline = new ExamplePipeline();
        webcam1.setPipeline(examplePipeline);

        webcam1.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                webcam1.startStreaming(1280, 720, OpenCvCameraRotation.UPRIGHT);
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
      /*  for (int i = 0; i < 200; i++) { // 200 iterations * 50 milliseconds = 10 seconds
            telemetry.addLine("Measuring Camara stream");
            telemetry.update();
            // Process frames and accumulate color values
            totalLeftAvg += examplePipeline.leftavgfin;
            totalRightAvg += examplePipeline.rightavgfin;
            frameCount++;

            // Sleep for 50 milliseconds
            sleep(50);
        }*/

        left = examplePipeline.leftavgfin;
        right = examplePipeline.rightavgfin;

        telemetry.addLine("done computing");
        telemetry.update();
        // Average color values over ten seconds
        double averageLeft = left;//totalLeftAvg / frameCount;
        double averageRight = right;//totalRightAvg / frameCount;

        telemetry.addLine("Returning Values");
        telemetry.update();
        // Use the average values to determine autonomous steps





        if (left > right && (Math.abs(left - right)) >= 1.5) {
            zone = 1;

            Trajectory traj1 = drive.trajectoryBuilder(startPose)
                    // .forward(25)
                    .splineTo(new Vector2d(-10, 28), Math.toRadians(-7))
                    .build();
            Trajectory traj2 = drive.trajectoryBuilder(traj1.end())

                    .splineTo(new Vector2d(-10, 0), Math.toRadians(90))
                    .build();

            drive.followTrajectory(traj1);
            sleep(500);
            servo1.setPosition(0);
            sleep(1000);
            drive.followTrajectory(traj2);


        } else if (left < right && (Math.abs(left - right)) >= 1.5) {
            zone = 2;
            telemetry.addData("Zone", zone);
            telemetry.addData("Average Left Value", averageLeft);
            telemetry.addData("Average Right Value", averageRight);
            telemetry.update();



            Trajectory traj1 = drive.trajectoryBuilder(startPose)
                    // .forward(25)
                    .splineTo(new Vector2d(-5, 28), Math.toRadians(0))
                    .build();
            Trajectory traj2 = drive.trajectoryBuilder(traj1.end())

                    .splineTo(new Vector2d(-10, 0), Math.toRadians(90))
                    .build();

            drive.followTrajectory(traj1);
            sleep(500);
            servo1.setPosition(0);
            sleep(1000);
            drive.followTrajectory(traj2);


        } else {
            zone = 3;
            telemetry.addData("Zone", zone);
            telemetry.addLine("C");
            telemetry.update();

          //  spikeMarkX = 7;
          //  spikeMarkY = 28;
           // spikeMarkDegrees = 5;


            Trajectory traj1 = drive.trajectoryBuilder(startPose)
                    // .forward(25)
                    .splineTo(new Vector2d(28, 7), Math.toRadians(5))
                    .build();
            Trajectory traj2 = drive.trajectoryBuilder(traj1.end())

                    .splineTo(new Vector2d(0, -10), Math.toRadians(90))
                    .build();

            drive.followTrajectory(traj1);
            sleep(500);
            servo1.setPosition(0);
            sleep(1000);
            drive.followTrajectory(traj2);
            sleep(500);

        }




     /*   Trajectory traj3 = drive.trajectoryBuilder(afterSensing)

                .forward(90)
                //  .splineTo(new Vector2d(30, 80), Math.toRadians(0))
                .build();

        drive.followTrajectory(traj3);


*/

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

            Rect leftRect = new Rect(1, 1, 400, 500);
            Rect rightRect = new Rect(800, 1, 400, 500);//midile is 640

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
