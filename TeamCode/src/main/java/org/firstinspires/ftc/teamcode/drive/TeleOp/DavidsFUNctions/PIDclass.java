package org.firstinspires.ftc.teamcode.drive.TeleOp.DavidsFUNctions;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;


public class PIDclass  {

    Halt halt = new Halt();

    public double slideKp = 0.02;
    public double slideKi = 0;
    public double slideKd = 0;
    public double slideIntegralSum = 0;
    public double slideLastError = 0;
    //public final int CUTOFF = 20;

    public int minSlides = -100;
    public int maxSlides = 2700;

    public double out = 0;

    public int targetInputSlidesValue = 0;

    public void magicPID(DcMotor slide, int setpoint, ElapsedTime timer) {

        setpoint = Range.clip(setpoint, minSlides, maxSlides);//limit

        //while (opModeIsActive() && Math.abs(slide.getCurrentPosition() - setpoint) > CUTOFF) {

        // obtain the encoder position
        int encoderPosition = slide.getCurrentPosition();
        // calculate the error
        double error = setpoint - encoderPosition;

        // rate of change of the error
        double derivative = (error - slideLastError) / timer.seconds();

        // sum of all error over time
        slideIntegralSum = slideIntegralSum + (error * timer.seconds());

        out = (slideKp * error) + (slideKi * slideIntegralSum) + (slideKd * derivative);
        out = Range.clip(out, -1, 1);

        slide.setPower(out);

        slideLastError = error;

        timer.reset();

    }
    public void zero(DcMotor slide, ElapsedTime tima, TouchSensor touchy) {

        if(touchy.isPressed()){
            slide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            slide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            slide.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            slide.setPower(-0.1);

        }else{
            slide.setPower(-1);
        }

    }
}