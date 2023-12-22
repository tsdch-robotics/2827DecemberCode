package org.firstinspires.ftc.teamcode.drive.TeleOp.DavidsFUNctions;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

public class moveWithBasicEncoder {



    public void powerSlider(DcMotor slider, int targetPos) {//need to add time as a parameter and other computational values, otherwise nothing will happen to then


        slider.setTargetPosition(targetPos);
        slider.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        slider.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        slider.setPower(1);

    }

}