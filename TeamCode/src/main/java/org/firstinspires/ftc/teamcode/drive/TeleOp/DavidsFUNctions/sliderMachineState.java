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
    public static final int HIGHpos = 2500;
    //todo adjust these values

    public static final double stabOpen = 0;//servo pos
    public static final double stabClosed = 1;//servo pos

    public static final double wristThreaten = 0;//servo pos
    public static final double wristStab = 0.5;//servo pos
    public static final double wristScore = 1;//servo pos


    public static final double armThreaten = 0;//servo pos
    public static final double armStab = 0.5;//servo pos
    public static final double armScore = 1;//servo pos




    public void magicalMacro (DcMotor slider, Servo arm1, Servo arm2,
                              Servo wrist, Servo stabberLeft,
                              Servo stabberRight, slidePosition targetMachineState){

        switch (targetMachineState) {
            case THREATEN:

                stabberLeft.setPosition(stabOpen);
                stabberRight.setPosition(stabOpen);
                magicalMachine.powerSlider(slider, THREATENINGpos);
                wrist.setPosition(wristThreaten);
                arm1.setPosition(armThreaten);
                arm2.setPosition(armThreaten);//need to make servos go the same direction

                break;


            case STAB:

                magicalMachine.powerSlider(slider, STABBINGpos);
                wrist.setPosition(wristStab);
                arm1.setPosition(armStab);
                arm2.setPosition(armStab);
                stabberLeft.setPosition(stabClosed);
                stabberRight.setPosition(stabClosed);

                break;
            case LOW:


                magicalMachine.powerSlider(slider, LOWpos);
                wrist.setPosition(wristScore);
                arm1.setPosition(armScore);
                arm2.setPosition(armScore);

                break;
            case MEDIUM:

                magicalMachine.powerSlider(slider, MEDIUMpos);
                wrist.setPosition(wristScore);
                arm1.setPosition(armScore);
                arm2.setPosition(armScore);


                break;
            case HIGH:

                magicalMachine.powerSlider(slider, HIGHpos);
                wrist.setPosition(wristScore);
                arm1.setPosition(armScore);
                arm2.setPosition(armScore);

                break;



        }



    }



}
