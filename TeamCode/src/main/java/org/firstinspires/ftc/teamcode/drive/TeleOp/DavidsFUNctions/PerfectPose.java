
package org.firstinspires.ftc.teamcode.drive.TeleOp.DavidsFUNctions;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PIDCoefficients;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;


import org.firstinspires.ftc.teamcode.drive.TeleOp.DavidsFUNctions.sliderMachineState;
import org.firstinspires.ftc.teamcode.drive.TeleOp.DavidsFUNctions.moveWithBasicEncoder;

import org.firstinspires.ftc.teamcode.drive.TeleOp.DavidsFUNctions.Halt;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

@Config
@Disabled
@TeleOp(name = "PerfectPose", group = "TeleOp")
public class PerfectPose extends OpMode {

    //for finding the perfect positions of servos

    sliderMachineState executeSlides = new sliderMachineState();
    private DcMotor frontLeftMotor;
    private DcMotor frontRightMotor;
    private DcMotor rearLeftMotor;
    private DcMotor rearRightMotor;
    private DcMotor slides;
    private DcMotor intake;
    private Servo arm1;
    private Servo arm2;

    private Servo wrist;
    private Servo finger1;

    private Servo finger2;


    public static double currentFinger1 = .5;//servo pos
    public static double currentFinger2 = .5;//servo pos

    public static double currentWrist = .5;//servo pos
    public static double currentArm = 0;//servo pos
    //public static double currentArm2 = .5;


    @Override
    public void init() {

        /*
    public static final double stabOpen = 0;//servo pos
    public static final double stabClosed = 1;//servo pos

    public static final double wristThreaten = 0;//servo pos
    public static final double wristStab = 0.5;//servo pos
    public static final double wristScore = 1;//servo pos


    public static final double armThreaten = 0;//servo pos
    public static final double armStab = 0.5;//servo pos
    public static final double armScore = 1;//servo pos*/


        arm1 = hardwareMap.servo.get("arm1");
        arm2 = hardwareMap.servo.get("arm2");
        wrist = hardwareMap.servo.get("wrist");
        finger1 = hardwareMap.servo.get("finger1");
        finger2 = hardwareMap.servo.get("finger2");

        arm1.setDirection(Servo.Direction.REVERSE);

        frontLeftMotor = hardwareMap.dcMotor.get("FL");
        frontRightMotor = hardwareMap.dcMotor.get("FR");
        rearLeftMotor = hardwareMap.dcMotor.get("BL");
        rearRightMotor = hardwareMap.dcMotor.get("BR");

        intake = hardwareMap.dcMotor.get("intake");
        slides = hardwareMap.dcMotor.get("slides");
        slides.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        slides.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);



    }

    @Override
    public void loop() {

        arm1.setPosition(currentArm);
        arm2.setPosition(currentArm);
        wrist.setPosition(currentWrist);
        finger1.setPosition(currentFinger1);
        finger2.setPosition(currentFinger2);

        //telemetry
        telemetry.update();
        telemetry.addData("Position of slides", slides.getCurrentPosition());
        telemetry.addData("Position of arm1", arm1.getPosition());
        telemetry.addData("Position of arm2", arm2.getPosition());
        telemetry.addData("Position of finger1", finger1.getPosition());
        telemetry.addData("Position of finger2", finger2.getPosition());
        telemetry.addData("Position of wrist", wrist.getPosition());

    }
}