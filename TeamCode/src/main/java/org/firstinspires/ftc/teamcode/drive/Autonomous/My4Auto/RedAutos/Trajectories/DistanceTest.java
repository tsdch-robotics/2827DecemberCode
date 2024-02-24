package org.firstinspires.ftc.teamcode.drive.Autonomous.My4Auto.RedAutos.Trajectories;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DistanceSensor;

import org.firstinspires.ftc.teamcode.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

/*
 * This is an example of a more complex path to really test the tuning.
 */
@Disabled
@Autonomous(group = "drive")
public class DistanceTest extends LinearOpMode {


    public DistanceSensor tsL;
    public DistanceSensor tsR;

    public Trajectory traj;

    @Override
    public void runOpMode() throws InterruptedException {

        tsL = hardwareMap.get(DistanceSensor.class, "tsL");
        tsR = hardwareMap.get(DistanceSensor.class, "tsR");

        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);



        waitForStart();

        if (isStopRequested()) return;

        TrajectorySequence traj = drive.trajectorySequenceBuilder(new Pose2d())


                .waitSeconds(5)
                .distanceSensorStraightMove(0, tsL, tsR, new Pose2d(0, 0, Math.toRadians(0)),
                        SampleMecanumDrive.getVelocityConstraint(15, DriveConstants.MAX_ANG_VEL,
                                DriveConstants.TRACK_WIDTH), SampleMecanumDrive.getAccelerationConstraint((DriveConstants.MAX_ACCEL)))


                .build();

        drive.followTrajectorySequence(traj);

        //sleep(2000);

        /*drive.followTrajectory(
                drive.trajectoryBuilder(traj.end(), true)
                        .splineTo(new Vector2d(0, 0), Math.toRadians(180))
                        .build()
        );*/
    }
}