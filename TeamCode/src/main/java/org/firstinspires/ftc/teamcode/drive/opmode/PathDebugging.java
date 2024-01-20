package org.firstinspires.ftc.teamcode.drive.opmode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

/*
 * This is an example of a more complex path to really test the tuning.
 */
@Autonomous(group = "drive")
public class PathDebugging extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        Pose2d startPose = new Pose2d(-33, -60, Math.toRadians(90));
        drive.setPoseEstimate(startPose);

        waitForStart();

        if (isStopRequested()) return;

        TrajectorySequence trajectory1 = drive.trajectorySequenceBuilder(startPose)



                .lineToSplineHeading(new Pose2d(-49, -49, Math.toRadians(90)))

                .lineToSplineHeading(new Pose2d(-70, -30, Math.toRadians(90)))


                 .lineToSplineHeading(new Pose2d(-52, -40, Math.toRadians(0)))



                //.waitSeconds(1)


                .lineToSplineHeading(new Pose2d(-70, -40, Math.toRadians(90)))


                /*.lineToSplineHeading(new Pose2d(-47, -40, Math.toRadians(0)),
                        SampleMecanumDrive.getVelocityConstraint(30, DriveConstants.MAX_ANG_VEL,
                                DriveConstants.TRACK_WIDTH), SampleMecanumDrive.getAccelerationConstraint((DriveConstants.MAX_ACCEL))

                )*/

                /* .lineToSplineHeading(new Pose2d(-44, -70, Math.toRadians(0)),
                         SampleMecanumDrive.getVelocityConstraint(30, DriveConstants.MAX_ANG_VEL,
                                 DriveConstants.TRACK_WIDTH), SampleMecanumDrive.getAccelerationConstraint((DriveConstants.MAX_ACCEL))

                 )
*/

                .lineToSplineHeading(new Pose2d(-45, -70, Math.toRadians(0)))

                .lineToSplineHeading(new Pose2d(-45, -59, Math.toRadians(0)))



                .lineToSplineHeading(new Pose2d(14, -59, Math.toRadians(0)))



                .lineToSplineHeading(new Pose2d(35, -29, Math.toRadians(0)))

                .lineToSplineHeading(new Pose2d(45, -29, Math.toRadians(0)))

                .waitSeconds(1)




                .waitSeconds(1)

                .setReversed(true)




                .splineTo(new Vector2d(45, -60), Math.toRadians(-90))


                 .waitSeconds(5)


                .build();

        drive.followTrajectorySequence(trajectory1);

        sleep(2000);


    }
}