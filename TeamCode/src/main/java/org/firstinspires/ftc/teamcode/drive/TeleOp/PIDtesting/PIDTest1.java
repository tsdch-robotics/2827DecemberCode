package org.firstinspires.ftc.teamcode.drive.TeleOp.PIDtesting;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

@TeleOp
public class PIDTest1 extends LinearOpMode {

    public DcMotor slides;
    public double Kp = 0.02;
    public double Ki = 0;
    public double Kd = 0;
    public double integralSum = 0;
    public double lastError = 0;
    public final int CUTOFF = 20;

    public int minSlides = -100;
    public int maxSlides = 4000;

    public double out = 0;

    public int targetInputSlidesValue = 0;

    public ElapsedTime timer = new ElapsedTime();


    @Override
    public void runOpMode() {

        telemetry.addLine("opModeRun");
        telemetry.update();

        slides = hardwareMap.dcMotor.get("slides");

        timer.reset();

        waitForStart();

        while(opModeIsActive()) {


            if(gamepad1.a){
                targetInputSlidesValue = 400;
                telemetry.addLine("a was pressed!");
            }else if(gamepad1.b){
                targetInputSlidesValue = 1500;
                telemetry.addLine("b was pressed!");
            }
            else if(gamepad1.x){
                targetInputSlidesValue = 0;
                telemetry.addLine("b was pressed!");
            }
            else if(gamepad1.y){
                targetInputSlidesValue = 2000;
                telemetry.addLine("b was pressed!");
            }

            magicPID(slides, targetInputSlidesValue);

            telemetry.addData("slides val", slides.getCurrentPosition());
            telemetry.update();
           //do something

        }
    }

    public void magicPID(DcMotor slide, int setpoint) {

        setpoint = Range.clip(setpoint, minSlides, maxSlides);//limit
        telemetry.addLine("setPoint set with a range.clip!");

        //while (opModeIsActive() && Math.abs(slide.getCurrentPosition() - setpoint) > CUTOFF) {

            // obtain the encoder position
        int encoderPosition = slide.getCurrentPosition();
            // calculate the error
        double error = setpoint - encoderPosition;

            // rate of change of the error
        double derivative = (error - lastError) / timer.seconds();

            // sum of all error over time
        integralSum = integralSum + (error * timer.seconds());

        out = (Kp * error) + (Ki * integralSum) + (Kd * derivative);
        out = Range.clip(out, -1, 1);

        slide.setPower(out);

        lastError = error;

        timer.reset();
        telemetry.addLine("timer reset!");
        telemetry.addData("motor pos", slide.getCurrentPosition());
        telemetry.addData("Out Power:", out);;


    }
}