package org.firstinspires.ftc.teamcode.drive.Autonomous;

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
public class BlueRightSide extends LinearOpMode {
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

    public static double ID1spikeMarkX = 28;
    public static double ID1spikeMarkY = 6.5;

    public static double ID1spikeMarkDegrees = 5;

    public static double ID2spikeMarkX = 29;
    public static double ID2spikeMarkY = -4;

    public static double ID2spikeMarkDegrees = 0;

    public static double ID3spikeMarkX = 29;
    public static double ID3spikeMarkY = -12;

    public static double ID3spikeMarkDegrees = -5;

    public static double randomParameter = 4;










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


       /* double left = examplePipeline.leftavgfin;
        double right = examplePipeline.rightavgfin;






        telemetry.addLine("done computing");
        telemetry.addData("left", left);
        telemetry.addData("right", right);
        telemetry.update();
        sleep(3000);
        // Average color values over ten seconds
        double averageLeft = left;//totalLeftAvg / frameCount;
        double averageRight = right;//totalRightAvg / frameCount;

        telemetry.addLine("Returning Values");
        telemetry.update();
        // Use the average values to determine autonomous steps

*/

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



        if (averageRight > averageLeft && (Math.abs(averageRight - averageLeft)) >= randomParameter) {
            zone = 2;
            telemetry.addData("Zone", zone);
            telemetry.update();
//middle

            Trajectory traj1 = drive.trajectoryBuilder(startPose)
                    // .forward(25)
                    .splineTo(new Vector2d(ID2spikeMarkX, ID2spikeMarkY), Math.toRadians(ID2spikeMarkDegrees))
                    .build();


            Trajectory traj2 = drive.trajectoryBuilder(traj1.end())




                    .lineToLinearHeading(new Pose2d(1, 0, Math.toRadians(90)))
                    // .splineTo(new Vector2d(0, 0), Math.toRadians(-90))
                    .build();

            Trajectory traj3 = drive.trajectoryBuilder(traj2.end())



                    .forward(70)
                    // .splineTo(new Vector2d(0, 0), Math.toRadians(-90))
                    .build();

            drive.followTrajectory(traj1);
            sleep(500);
            flicker.setPosition(0);
            sleep(1000);
            drive.followTrajectory(traj2);
            drive.followTrajectory(traj3);
            sleep(500);


        } else if (averageRight < averageLeft && (Math.abs(averageRight - averageLeft)) >= randomParameter) {

            zone = 3;
            telemetry.addData("Zone", zone);
            telemetry.update();
//middle

            Trajectory traj1 = drive.trajectoryBuilder(startPose)
                    // .forward(25)
                    .splineTo(new Vector2d(ID3spikeMarkX, ID3spikeMarkY), Math.toRadians(ID3spikeMarkDegrees))
                    .build();


            Trajectory traj2 = drive.trajectoryBuilder(traj1.end())




                    .lineToLinearHeading(new Pose2d(1, 0, Math.toRadians(90)))
                    // .splineTo(new Vector2d(0, 0), Math.toRadians(-90))
                    .build();

            Trajectory traj3 = drive.trajectoryBuilder(traj2.end())



                    .forward(70)
                    // .splineTo(new Vector2d(0, 0), Math.toRadians(-90))
                    .build();

            drive.followTrajectory(traj1);
            sleep(500);
            flicker.setPosition(0);
            sleep(1000);
            drive.followTrajectory(traj2);
            drive.followTrajectory(traj3);

            sleep(500);

        } else {
            zone = 1;
            telemetry.addData("Zone", zone);

            telemetry.update();




            Trajectory traj1 = drive.trajectoryBuilder(startPose)
                    // .forward(25)
                    .splineTo(new Vector2d(ID1spikeMarkX, ID1spikeMarkY), Math.toRadians(ID1spikeMarkDegrees))
                    .build();
            Trajectory traj2 = drive.trajectoryBuilder(traj1.end())



                    .lineToLinearHeading(new Pose2d(24, 0, Math.toRadians(5)))
                    //.lineToLinearHeading(new Pose2d(1, 0, Math.toRadians(90)))
                   // .splineTo(new Vector2d(0, 0), Math.toRadians(-90))
                    .build();

            Trajectory traj3 = drive.trajectoryBuilder(traj2.end())




                    .lineToLinearHeading(new Pose2d(1, 0, Math.toRadians(90)))
                    // .splineTo(new Vector2d(0, 0), Math.toRadians(-90))
                    .build();

            Trajectory traj4 = drive.trajectoryBuilder(traj3.end())



                    .forward(70)
                    // .splineTo(new Vector2d(0, 0), Math.toRadians(-90))
                    .build();

            drive.followTrajectory(traj1);
            sleep(500);
            flicker.setPosition(0);
            sleep(1000);
            drive.followTrajectory(traj2);
            drive.followTrajectory(traj3);
            drive.followTrajectory(traj4);
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
//1280,720
            Rect leftRect = new Rect(200, 100, 400, 500);
            Rect rightRect = new Rect(800, 100, 400, 500);//midile is 640

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
