
package org.firstinspires.ftc.teamcode.drive.TeleOp.DavidsFUNctions;

import android.util.Size;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.hardware.ams.AMSColorSensor;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.ExposureControl;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.GainControl;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.firstinspires.ftc.teamcode.drive.TeleOp.DavidsFUNctions.Kalman2;


import java.util.Vector;
import java.util.concurrent.TimeUnit;

@Config
@TeleOp(name="", group="Linear OpMode")
public class NewAprilTagVision extends LinearOpMode {

    private ElapsedTime runtime = new ElapsedTime();

    Vector2d aprilTagKnownPosition = new Vector2d(0, 0);

    Kalman kalman = new Kalman();

    Vector2d rawVector;
    Vector2d robotVectorTagRelative;


    double x = 0;
    public static double Q = .1; // your model covariance.1
    public static double R = 2; // your sensor covariance.4
    public static double p = 1; // your initial covariance guess
    double K; // Kalman gain

    double x_previous = x;
    double p_previous = p;
    double u = 0;
    double z = 0;


    Vector2d robotVectorFieldRelative;
    Pose2d robotPoseFieldRelativeTagCalibrated;




    //for BLUE SIDE
    double aprilTagY = 0;
    double aprilTagX = 0;
    int cycle = 0;

    public int webcamOriginOffset = 6;

    @Override
    public void runOpMode() {


        Telemetry telemetry = new MultipleTelemetry(this.telemetry, FtcDashboard.getInstance().getTelemetry());


        AprilTagProcessor tagProcessor = new AprilTagProcessor.Builder()

                .setDrawAxes(true)
                .setDrawCubeProjection(true)
                .setDrawTagID(true)
                .setDrawTagOutline(true)
                .setLensIntrinsics(932.381, 932.381, 338.397,242.298)
                .build();


        VisionPortal visionPortal = new VisionPortal.Builder()
                .addProcessor(tagProcessor)
                .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
                .setCameraResolution(new Size(640, 480))

                .build();

        waitForStart();
        runtime.reset();

        while (visionPortal.getCameraState() != VisionPortal.CameraState.STREAMING){}

        ExposureControl exposure = visionPortal.getCameraControl(ExposureControl.class);
        exposure.setMode(ExposureControl.Mode.Manual);
        exposure.setExposure(15, TimeUnit.MILLISECONDS);

        GainControl gain = visionPortal.getCameraControl(GainControl.class);
        gain.setGain(255);

        while (opModeIsActive()) {

            //tagProcessor.setPoseSolver(AprilTagProcessor.PoseSolver.APRILTAG_BUILTIN);

            if (tagProcessor.getDetections().size() > 0){
                AprilTagDetection tag = tagProcessor.getDetections().get(0);

                cycle += 1;
                double yaw = -tag.ftcPose.yaw;
                double xValue = tag.ftcPose.x;
                double yValue = tag.ftcPose.y;

                //y value + origin offset

                double vectorRotatedXvalue = xValue * Math.cos(Math.toRadians(yaw)) - yValue * Math.sin(Math.toRadians(yaw));
                double vectorRotatedYValue = xValue * Math.sin(Math.toRadians(yaw)) + yValue * Math.cos(Math.toRadians(yaw));

                robotVectorTagRelative = new Vector2d(vectorRotatedXvalue, vectorRotatedYValue);


                telemetry.addData("vector:", robotVectorTagRelative);

                //robotPoseFieldRelativeTagCalibrated = new Pose2d(robotVectorTagRelative, Math.toRadians(((yaw))));


                telemetry.addData("tag Calibrated X:", vectorRotatedXvalue);
                telemetry.addData("tag Calibrated Y:", vectorRotatedYValue);

                telemetry.addData("x", xValue);
                telemetry.addData("y", yValue);
                telemetry.addData("yaw", yaw);
                //telemetry.addData("roll", tag.ftcPose.roll);
                //telemetry.addData("pitch", tag.ftcPose.pitch);
                //telemetry.addData("bearing", tag.ftcPose.bearing);


                //telemetry.addData("cycle", cycle);

            }

            telemetry.update();

        }
    }
}