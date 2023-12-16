package org.firstinspires.ftc.teamcode.drive.TeleOp.DavidsFUNctions;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

public class slidesPIDpower {

    private double slideKp = 0.1; // Proportional gain
    private double slideKi = 0.01; // Integral gain
    private double slideKd = 0.001; // Derivative gain
    private int slideTargetPosition = 0;
    private int slideError, slideIntegral, slideDerivative, slideLastError;
    private double slideCorrection, slidePower;
    private ElapsedTime slideTimer = new ElapsedTime();

    public double powerSlider(DcMotor slider, int targetPos) {

        // Calculate PID components
        slideError = slideTargetPosition - slider.getCurrentPosition();
        slideIntegral += slideError;
        slideDerivative = slideError - slideLastError;

        // Calculate PID correction
        double dt = slideTimer.seconds();
        slideTimer.reset();
        slideCorrection = (slideKp * slideError + slideKi * slideIntegral * dt + slideKd * slideDerivative / dt);

        // Calculate motor power
        slidePower = Range.clip(slideCorrection, -1, 1);
        slideLastError = slideError;//?

        return slidePower;

    }

}