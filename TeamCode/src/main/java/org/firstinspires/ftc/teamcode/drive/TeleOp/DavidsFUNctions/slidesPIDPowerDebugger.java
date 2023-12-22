
package org.firstinspires.ftc.teamcode.drive.TeleOp.DavidsFUNctions;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PIDCoefficients;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.Range;


import org.firstinspires.ftc.teamcode.drive.TeleOp.DavidsFUNctions.sliderMachineState;
import org.firstinspires.ftc.teamcode.drive.TeleOp.DavidsFUNctions.slidesPIDpower;

import org.firstinspires.ftc.teamcode.drive.TeleOp.DavidsFUNctions.Halt;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

@TeleOp(name = "slidesPIDPowerDebugger", group = "TeleOp")
public class slidesPIDPowerDebugger extends OpMode {

//custom funcitons, used to save code space

    public static double slideKp = 0.1; // Proportional gain
    public static double slideKi = 0.01; // Integral gain
    public static double slideKd = 0.001; // Derivative gain
    private int slideTargetPosition;
    private int slideError, slideIntegral, slideDerivative, slideLastError;
    private double slideCorrection, slidePower;
    private ElapsedTime slideTimer = new ElapsedTime();

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

    private TouchSensor touch;

    @Override
    public void init() {

        arm1 = hardwareMap.servo.get("arm1");
        arm2 = hardwareMap.servo.get("arm2");
        wrist = hardwareMap.servo.get("wrist");
        finger1 = hardwareMap.servo.get("finger1");
        finger2 = hardwareMap.servo.get("finger2");

        frontLeftMotor = hardwareMap.dcMotor.get("FL");
        frontRightMotor = hardwareMap.dcMotor.get("FR");
        rearLeftMotor = hardwareMap.dcMotor.get("BL");
        rearRightMotor = hardwareMap.dcMotor.get("BR");
        intake = hardwareMap.dcMotor.get("intake");
        slides = hardwareMap.dcMotor.get("slides");
        //slides.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        slides.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        touch = hardwareMap.get(TouchSensor.class, "touch");
//because we are using PID and not their dinky encoder stuff

    }

    @Override
    public void loop() {
        //telemetry
        telemetry.addData("Position of slides", slides.getCurrentPosition());
        telemetry.addData("Power of slides,", slides.getPower());
        telemetry.update();


        if(gamepad1.a){
            slideTargetPosition = 2000;
        }else if(gamepad1.y){
            slideTargetPosition = 0;
        }else{
            slideTargetPosition = 500;
        }

        if(touch.isPressed()){
            slides.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            telemetry.addData("Calibrated to 0 position. SLide positin:", slides.getCurrentPosition());
        }//if it hits the touch sensor it will stay at 0 and reset


        // calculate pid stuff
        slideError = slideTargetPosition - slides.getCurrentPosition();
        slideIntegral += slideError;
        slideDerivative = slideError - slideLastError;

        // calculate pdi correction
        double dt = slideTimer.seconds();//dt means delta time
        slideTimer.reset();
        slideCorrection = (slideKp * slideError + slideKi * slideIntegral * dt + slideKd * slideDerivative / dt);

        // calculate motor power based on correction
        slidePower = Range.clip(slideCorrection, -1, 1);//keep that number within range
        slideLastError = slideError;//?

        slides.setPower(slidePower);



    }
}