package org.firstinspires.ftc.teamcode.drive.opmode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

/*
 * This is an example of a more complex path to really test the tuning.
 */
@Autonomous(group = "drive")
@Disabled
public class SplineTest extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        waitForStart();

        if (isStopRequested()) return;

        TrajectorySequence traj = drive.trajectorySequenceBuilder(new Pose2d(15, 60, Math.toRadians(270)))

                .splineTo(new Vector2d(14, 33), Math.toRadians(-90))//middle pos
                .waitSeconds(.5)

                .lineToSplineHeading(new Pose2d(15, 45, Math.toRadians(-80)))
                .waitSeconds(.25)




                .splineTo(new Vector2d(47, 35), Math.toRadians(0))//to board

                .waitSeconds(1)


                .setReversed(true)//intaking side is the front now
                .splineTo(new Vector2d(45, 58), Math.toRadians(90))



                .build();





        drive.followTrajectorySequence(traj);

        sleep(2000);

        drive.followTrajectory(
                drive.trajectoryBuilder(traj.end(), true)
                        //.splineTo(new Vector2d(0, 0), Math.toRadians(180))
                        .build()
        );
    }
}
