package org.firstinspires.ftc.teamcode.drive.Autonomous.My4Auto.RedAutos.Trajectories;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.drive.Autonomous.My4Auto.RedAutos.RedRightScoreStack;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.drive.TeleOp.DavidsFUNctions.moveWithBasicEncoder;
import org.firstinspires.ftc.teamcode.drive.TeleOp.DavidsFUNctions.sliderMachineState;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.openftc.easyopencv.OpenCvWebcam;

/*
 * This is an example of a more complex path to really test the tuning.
 */

//@Autonomous(group = "drive")
public class RRZ1_SCR_PR_YW {

    DistanceSensor tsL;
    DistanceSensor tsR;

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

    private Servo intakeLeft;
    private Servo intakeRight;

    public boolean greaterThanTargetPercentRedPixels1 = false;
    public boolean greaterThanTargetPercentRedPixels2 = false;

    private double targetPixPercent1 = .2;
    private double targetPixPercent2 = .2;

    private DcMotor slides;
    private DcMotor intake;

    private Servo arm1;
    private Servo arm2;
    private Servo wrist;
    private Servo finger1;
    private Servo finger2;
    private Servo flicker;


    public TrajectorySequence moveToSensingPos(SampleMecanumDrive robot, Pose2d startPose){

        TrajectorySequence traj1 = robot.trajectorySequenceBuilder(startPose)


                .build();



        return traj1;

    }


   /* @Override
    public void runOpMode() throws InterruptedException {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);


        Pose2d startPose = new Pose2d(14.5, -60, Math.toRadians(90));
        drive.setPoseEstimate(startPose);

        waitForStart();

        if (isStopRequested()) return;

        Trajectory traj = drive.trajectoryBuilder(new Pose2d())
                .splineTo(new Vector2d(30, 30), 0)
                .build();



        drive.followTrajectory(traj);

        sleep(2000);

        drive.followTrajectory(
                drive.trajectoryBuilder(traj.end(), true)
                        .splineTo(new Vector2d(0, 0), Math.toRadians(180))
                        .build()
        );
    }*/
}