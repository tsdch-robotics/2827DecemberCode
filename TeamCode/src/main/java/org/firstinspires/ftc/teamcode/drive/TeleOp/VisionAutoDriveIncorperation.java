package org.firstinspires.ftc.teamcode.drive.TeleOp;

import android.util.Size;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.acmerobotics.roadrunner.util.Angle;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.ExposureControl;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.GainControl;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.drive.TeleOp.DavidsFUNctions.Kalman;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.concurrent.TimeUnit;

@TeleOp(name = "VisionAutoDriveIncorperation", group = "TeleOp")
public class VisionAutoDriveIncorperation extends LinearOpMode {
    // Define 2 states, drive control or automatic control
    enum Mode {
        DRIVER_CONTROL,
        AUTOMATIC_CONTROL
    }

    Mode currentMode = Mode.DRIVER_CONTROL;

    // The coordinates we want the bot to automatically go to when we press the A button
    Vector2d targetAVector = new Vector2d(45, 45);
    // The heading we want the bot to end on for targetA
    double targetAHeading = Math.toRadians(90);

    // The location we want the bot to automatically go to when we press the B button
    Vector2d targetBVector = new Vector2d(-15, 25);

    // The angle we want to align to when we press Y
    double targetAngle = Math.toRadians(45);



    //VISIOn:
    //TODO: adjust april tag known cordinates, webcam orgin offset

    private ElapsedTime runtime = new ElapsedTime();

    Vector2d aprilTagKnownPosition = new Vector2d(0, 0);

    Kalman kalman = new Kalman();

    Vector2d rawVector;
    Vector2d robotVectorTagRelative;

    Vector2d robotVectorFieldRelative;
    Pose2d robotPoseFieldRelativeTagCalibrated;

    //for BLUE SIDE
    double aprilTagY = 0;
    double aprilTagX = 0;
    int cycle = 0;


    public int webcamOriginOffset = 6;

    boolean enableAuto = false;


    @Override
    public void runOpMode() throws InterruptedException {
        // Initialize custom cancelable SampleMecanumDrive class


        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        // We want to turn off velocity control for teleop
        // Velocity control per wheel is not necessary outside of motion profiled auto
        drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // Retrieve our pose from the PoseStorage.currentPose static field
        // See AutoTransferPose.java for further details


        Pose2d startPose = new Pose2d(20, -60, Math.toRadians(0));
        drive.setPoseEstimate(startPose);


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
        if (isStopRequested()) return;

        while (visionPortal.getCameraState() != VisionPortal.CameraState.STREAMING){}
        //safty feture to ensure that the camera is streaming before continuing
        ExposureControl exposure = visionPortal.getCameraControl(ExposureControl.class);
        exposure.setMode(ExposureControl.Mode.Manual);
        exposure.setExposure(15, TimeUnit.MILLISECONDS);

        GainControl gain = visionPortal.getCameraControl(GainControl.class);
        gain.setGain(255);
        //exposure and gain
        //todo! adjust exposure and gain


        while (opModeIsActive() && !isStopRequested()) {
            if (tagProcessor.getDetections().size() > 0){

                visionPortal.saveNextFrameRaw("Snapshot of last tag in match");
                AprilTagDetection tag = tagProcessor.getDetections().get(0);

                if (tag.id == 2){
                    enableAuto = true;
                    cycle += 1;
                    double yaw = -tag.ftcPose.yaw;
                    double xValue = tag.ftcPose.x;
                    double yValue = tag.ftcPose.y + webcamOriginOffset;//offset, this might not work

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
                    telemetry.update();
                }else{
                    telemetry.addLine("you detected the wrong tag");
                    telemetry.update();
                    enableAuto = false;
                }
            }else{
                telemetry.addLine("no tag detected");
                telemetry.update();
                enableAuto = false;
            }
//Ends

            // Update the drive class
            drive.update();

            // Read pose
            Pose2d poseEstimate = drive.getPoseEstimate();

            // Print pose to telemetry
            telemetry.addData("mode", currentMode);
            telemetry.addData("x", poseEstimate.getX());
            telemetry.addData("y", poseEstimate.getY());
            telemetry.addData("heading", poseEstimate.getHeading());
            telemetry.update();

            // We follow different logic based on whether we are in manual driver control or switch
            // control to the automatic mode
            switch (currentMode) {
                case DRIVER_CONTROL:
                    drive.setWeightedDrivePower(
                            new Pose2d(
                                    -gamepad1.left_stick_y,
                                    -gamepad1.left_stick_x,
                                    -gamepad1.right_stick_x
                            )
                    );

                    if (gamepad1.a) {
                        // If the A button is pressed on gamepad1, we generate a splineTo()
                        // trajectory on the fly and follow it
                        // We switch the state to AUTOMATIC_CONTROL

                        Trajectory traj1 = drive.trajectoryBuilder(poseEstimate)
                                .splineTo(targetAVector, targetAHeading)
                                .build();

                        drive.followTrajectoryAsync(traj1);

                        currentMode = Mode.AUTOMATIC_CONTROL;
                    } else if (gamepad1.b) {
                        // If the B button is pressed on gamepad1, we generate a lineTo()
                        // trajectory on the fly and follow it
                        // We switch the state to AUTOMATIC_CONTROL

                        Trajectory traj1 = drive.trajectoryBuilder(poseEstimate)
                                .lineTo(targetBVector)
                                .build();

                        drive.followTrajectoryAsync(traj1);

                        currentMode = Mode.AUTOMATIC_CONTROL;
                    } else if (gamepad1.y) {
                        // If Y is pressed, we turn the bot to the specified angle to reach
                        // targetAngle (by default, 45 degrees)

                        drive.turnAsync(Angle.normDelta(targetAngle - poseEstimate.getHeading()));

                        currentMode = Mode.AUTOMATIC_CONTROL;
                    }
                    break;
                case AUTOMATIC_CONTROL:
                    // If x is pressed, we break out of the automatic following
                    if (gamepad1.x) {
                        drive.cancelFollowing();//i made this function. It may not work. Found in smapleMecnaumDriveded.
                        currentMode = Mode.DRIVER_CONTROL;
                    }

                    // If drive finishes its task, cede control to the driver
                    if (!drive.isBusy()) {
                        currentMode = Mode.DRIVER_CONTROL;
                    }
                    break;
            }
        }
    }
}