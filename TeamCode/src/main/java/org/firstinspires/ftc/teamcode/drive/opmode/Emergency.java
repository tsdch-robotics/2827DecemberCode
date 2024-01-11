package org.firstinspires.ftc.teamcode.drive.opmode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

/*
 * This is an example of a more complex path to really test the tuning.
 */
@Autonomous(group = "drive", preselectTeleOp = "TeleOp2")

public class Emergency extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        waitForStart();

        if (isStopRequested()) return;

        TrajectorySequence traj = drive.trajectorySequenceBuilder(new Pose2d(-32.5, 60, Math.toRadians(-90)))

                .lineToSplineHeading(new Pose2d(-37, 45, Math.toRadians(-90)))
                .lineToSplineHeading(new Pose2d(-37, 50, Math.toRadians(0)))

                .lineToSplineHeading(new Pose2d(-35, 10, Math.toRadians(0)))


                .lineToSplineHeading(new Pose2d(55, 10, Math.toRadians(0)))



                .build();





        drive.followTrajectorySequence(traj);

        sleep(2000);


    }
}
