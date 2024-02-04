
package org.firstinspires.ftc.teamcode.drive.TeleOp;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.PIDCoefficients;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.robotcore.hardware.TouchSensor;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

import org.firstinspires.ftc.teamcode.drive.Autonomous.Misc.PoseStorage;
import org.firstinspires.ftc.teamcode.drive.TeleOp.DavidsFUNctions.PIDclass;
import org.firstinspires.ftc.teamcode.drive.TeleOp.DavidsFUNctions.PIDhangClass;
import org.firstinspires.ftc.teamcode.drive.TeleOp.DavidsFUNctions.sliderMachineState;

import org.firstinspires.ftc.teamcode.drive.TeleOp.DavidsFUNctions.Halt;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

@Config
@TeleOp(name = "Hang Adjusting", group = "TeleOp")
public class SlideManualSet extends OpMode {

//custom funcitons, used to save code space


    private DcMotor hang1;
    private DcMotor hang2;



    @Override
    public void init() {



        hang1 = hardwareMap.dcMotor.get("hang1");
        hang2 = hardwareMap.dcMotor.get("hang2");



        hang2.setDirection(DcMotorSimple.Direction.REVERSE);



        //hang2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        //hang1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        hang2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        hang2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        hang1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        hang1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);



    }

    @Override
    public void loop() {


        hang1.setPower(-gamepad2.left_stick_y);
        hang2.setPower(-gamepad2.right_stick_y);

    }

}