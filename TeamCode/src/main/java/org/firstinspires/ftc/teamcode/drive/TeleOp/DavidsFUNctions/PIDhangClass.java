package org.firstinspires.ftc.teamcode.drive.TeleOp.DavidsFUNctions;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;


public class PIDhangClass  {

    public double hangKp = 0.02;
    public double hangKi = 0;
    public double hangKd = 0;
    public double hangIntegralSum = 0;
    public double hangLastError = 0;
    //public final int CUTOFF = 20;

    public int minHang = -100;
    public int maxHang = 4500;

    public double out = 0;

    public int targetInputSlidesValue = 0;

    public void magicPID(DcMotor slide, int setpoint, ElapsedTime timer) {

        setpoint = Range.clip(setpoint, minHang, maxHang);//limit

        //while (opModeIsActive() && Math.abs(slide.getCurrentPosition() - setpoint) > CUTOFF) {

        // obtain the encoder position
        int encoderPosition = slide.getCurrentPosition();
        // calculate the error
        double error = setpoint - encoderPosition;

        // rate of change of the error
        double derivative = (error - hangLastError) / timer.seconds();

        // sum of all error over time
        hangIntegralSum = hangIntegralSum + (error * timer.seconds());

        out = (hangKp * error) + (hangKi * hangIntegralSum) + (hangKd * derivative);
        out = Range.clip(out, -1, 1);

        slide.setPower(out);

        hangLastError = error;

        timer.reset();

    }
}