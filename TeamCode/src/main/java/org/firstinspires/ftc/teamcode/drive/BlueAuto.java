package org.firstinspires.ftc.teamcode.drive;

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

/*
 * This is an example of a more complex path to really test the tuning.
 */
@Autonomous(group = "drive")
public class BlueAuto extends LinearOpMode {

    OpenCvWebcam webcam1 = null;

    @Override
    public void runOpMode() throws InterruptedException {
        Hardware sense = new Hardware();
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        // We want to start the bot at x: 10, y: -8, heading: 90 degrees
        Pose2d startPose = new Pose2d(0, 0, Math.toRadians(0));

        drive.setPoseEstimate(startPose);


        waitForStart();


        WebcamName webcamName = hardwareMap.get(WebcamName.class, "webcam1");
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam1 = OpenCvCameraFactory.getInstance().createWebcam(webcamName, cameraMonitorViewId);


        sense.webcam1.setPipeline(sense.new examplePipeline());

        sense.webcam1.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {//?

            @Override
            public void onOpened() {
                sense.webcam1.startStreaming(1920, 1080, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode) {
                // Handle error
            }
        });



        int numReadings = 10;
        int totalPosition = 0;

        for (int i = 0; i < numReadings; i++) {
            totalPosition += sense.getPosition();
            sleep(100); // Adjust the sleep time based on your needs
        }

        int averagePosition = totalPosition / numReadings;

        telemetry.addData("Average Position", averagePosition);
        telemetry.update();





        Trajectory traj1 = drive.trajectoryBuilder(startPose, true)
                .splineTo(new Vector2d(-40, -9), Math.toRadians(0))
                .build();

        Trajectory traj2 = drive.trajectoryBuilder(traj1.end())
                .splineTo(new Vector2d(0, 0), Math.toRadians(0))
                .build();

        drive.followTrajectory(traj1);
        drive.followTrajectory(traj2);

     /*   Hardware sense = new Hardware();



        WebcamName webcamName = hardwareMap.get(WebcamName.class, "webcam1");
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam1 = OpenCvCameraFactory.getInstance().createWebcam(webcamName, cameraMonitorViewId);






      //  sense.init();
        // sense.


  //      if (isStopRequested()) return;

        sense.webcam1.setPipeline(sense.new examplePipeline());

        sense.webcam1.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                sense.webcam1.startStreaming(720, 720, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode) {
                // Handle error
            }
        });

        int numReadings = 10;
        int totalPosition = 0;

        waitForStart();
        for (int i = 0; i < numReadings; i++) {
            totalPosition += sense.getPosition();
            sleep(10); // Adjust the sleep time based on your needs
        }

        double averagePosition = totalPosition / numReadings;
        int roundedAverage = (int) Math.round(averagePosition);

        telemetry.addData("Average Position", roundedAverage);
        telemetry.update();
*/



    }
}
