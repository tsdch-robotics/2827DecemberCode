package org.firstinspires.ftc.teamcode.drive.TeleOp.DavidsFUNctions;

import android.transition.Slide;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.drive.TeleOp.DavidsFUNctions.slidesPIDpower;


public class sliderMachineState {

    slidesPIDpower magicalMachine = new slidesPIDpower();

   public enum slidePosition {

        THREATEN, //waiting to stab
        STAB, //stabing


        LOW, //500
        MEDIUM,//1500
        HIGH;//2500
    }

    public static final int LOWpos = 500;
    public static final int THREATENINGpos = 100;

    public static final int midSTABpos = 300;
    public static final int STABBINGpos = 10;
    public static final int MEDIUMpos = 1500;

    public static final double HIGHpos = 2500;

    public void magicalMacro (DcMotor slider, Servo arm1, Servo arm2,
                              Servo wrist, Servo stabberLeft,
                              Servo stabberRight, slidePosition targetMachineState){

        switch (targetMachineState) {
            case THREATEN:

                magicalMachine.powerSlider(slider, THREATENINGpos);
                wrist.setPosition();
                arm1.setPosition();
                arm2.setPosition();
                stabberLeft.setPosition();
                stabberRight.setPosition();

                break;



            case STAB:

                magicalMachine.powerSlider(slider, STABBINGpos);
                wrist.setPosition();
                arm1.setPosition();
                arm2.setPosition();
                stabberLeft.setPosition();
                stabberRight.setPosition();

                break;
            case LOW:


                magicalMachine.powerSlider(slider, LOWpos);
                wrist.setPosition();
                arm1.setPosition();
                arm2.setPosition();
                stabberLeft.setPosition();
                stabberRight.setPosition();

                break;
            case MEDIUM:

                magicalMachine.powerSlider(slider, MEDIUMpos);
                wrist.setPosition();
                arm1.setPosition();
                arm2.setPosition();
                stabberLeft.setPosition();
                stabberRight.setPosition();

                break;
            case HIGH:

                magicalMachine.powerSlider(slider, HIGHpos);
                wrist.setPosition();
                arm1.setPosition();
                arm2.setPosition();
                stabberLeft.setPosition();
                stabberRight.setPosition();

                break;



        }



    }



}
